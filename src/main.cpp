
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <cmath>
#include "config.h"
#include "CameraFunctions/segmentation.h"
#include "CameraFunctions/line_detection.h"
#include "CameraFunctions/camera_setup.h"
#include "tcp_connect.h"
//#include "include/camera_setup.h"
//#include "include/color_segmentation.h"

#include <thread>
#include <atomic>
#include <mutex>

std::atomic<bool> running(true); // Flag to control the capture loop
cv::Mat latest_frame; // Shared frame variable
std::mutex frame_mutex; // Mutex for synchronizing access to the frame

float fps_capture_frames;
float fps_process_frame;
std::vector<cv::Point> leftLine;
std::vector<cv::Point> rightLine;

void capture_frames(cv::VideoCapture& cap) {
    
    double fps;
    int frame_count = 0;
    double time_start = cv::getTickCount();

    while (running) {
        #if ENABLE_READING_FROM_JPG == 1
            //std::string imagePath = "ogImage.jpg";
            //std::string imagePath = "ogCurves.jpg";
            //std::string imagePath = "ogCurves1.jpg";
            //std::string imagePath = "imagineSala1.jpg";
            //std::string imagePath = "imagineSala2.jpg";
            //std::string imagePath = "imagineSala3.jpg";
            //std::string imagePath = "imagineSala4.jpg";
            //std::string imagePath = "lines4.jpeg";
            //std::string imagePath = "lines5.jpeg";
            //std::string imagePath = "imagine12022024_02.jpg";
            //std::string imagePath = "imagine12022024_03.jpg";
            //std::string imagePath = "imagineThreshold_01.jpg";
            //std::string imagePath = "imagineThreshold_02.jpg";
            //std::string imagePath = "imagineThreshold_03.jpg";
            std::string imagePath = "imagineThreshold_04.jpg";

            cv::Mat frame = cv::imread(imagePath, cv::IMREAD_COLOR);
        #endif
        #if ENABLE_STREAMING
            cv::Mat frame;
            cap >> frame;
        #endif
        if (frame.empty()) break;
        
        frame = resizeImage(frame, 320, 180);
        //saveImage("imagine12022024_03.jpg")
        cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
        // TO CROP
        //frame = cropFrameTop(frame, 40);
        
        {
            std::lock_guard<std::mutex> lock(frame_mutex);
            latest_frame = frame.clone(); // Update the latest frame
        }

        frame_count++;
        double time_elapsed = (cv::getTickCount() - time_start) / cv::getTickFrequency();
        if (time_elapsed >= 1.0) {  // Update FPS every second
            fps = frame_count / time_elapsed;
            frame_count = 0;
            time_start = cv::getTickCount();
        }
        fps_capture_frames = fps;
    }
}

void process_frame() {
    
    double fps;
    int frame_count = 0;
    double time_start = cv::getTickCount();
    TcpConnection laptopTCP(9999);

    while (running) {
        cv::Mat frame;

        {
            std::lock_guard<std::mutex> lock(frame_mutex);
            frame = latest_frame.clone(); // Always copy the latest frame
        }
        
        // Process the frame if it is not empty (mainly for the first iterations)
        if (!frame.empty()) {
            // Calculate FPS
            frame_count++;
            double time_elapsed = (cv::getTickCount() - time_start) / cv::getTickFrequency();
            if (time_elapsed >= 1.0) {  // Update FPS every second
                fps = frame_count / time_elapsed;
                frame_count = 0;
                time_start = cv::getTickCount();
            }

            std::cout << "FPS(process_frame): " << fps << " FPS(capture_frame): " << fps_capture_frames << std::endl;

            cv::Mat skeleton = segmentEdges(frame);  
            //laptopTCP.sendFrame(skeleton);

            std::vector<std::vector<cv::Point>> lines = findLines(skeleton);

            cv::Mat outputImage = cv::Mat::zeros(frame.size(), CV_8UC3);  // Output image for drawing
      
            drawLines(outputImage,lines,cv::Scalar(255, 0, 0));
            //laptopTCP.sendFrame(outputImage);

            for (int i = 0; i < lines.size(); i++){
                lines[i] = fitPolinomial(lines[i],false);
                lines[i] = evenlySpacePoints(lines[i], num_points);
            }

            drawLines(outputImage,lines,cv::Scalar(0, 0, 255));
            laptopTCP.sendFrame(outputImage);

            for (int i = 0; i < lines.size(); i++) {
                lines[i] = removeHorizontalIf90Turn(lines[i]);
            }
            
            getLeftRightLines(lines,leftLine,rightLine);
            
            #if ENABLE_CALIBRATE_CAMERA == 1
                // TO CALIBRATE CAMERA
                cv::Point result;
                std::vector<cv::Point> interpolatedPoints;
                try {
                    interpolatedPoints.push_back(interpolateClosestPoints(leftLine, 50));
                    interpolatedPoints.push_back(interpolateClosestPoints(leftLine, 130));
                    interpolatedPoints.push_back(interpolateClosestPoints(rightLine, 50));
                    interpolatedPoints.push_back(interpolateClosestPoints(rightLine, 130));

                    // Write interpolated points to a TXT file
                    writePointsToTxt(interpolatedPoints, "interpolated_points.txt");

                } catch (const std::exception& e) {
                    std::cerr << "Error: " << e.what() << "\n";
                }
            #endif     
            
            std::vector<cv::Point> allMidpoints = findMiddle(leftLine,rightLine,widthBirdsEyeView,heightBirdsEyeView);
            // Create a frame of the desired size
            cv::Size frameSize(widthBirdsEyeView, heightBirdsEyeView);

            // Initialize an empty image (black by default)
            cv::Mat birdEyeViewWithPoints = cv::Mat::zeros(frameSize, CV_8UC3); // 3 channels (color)
            std::cout <<  "dstPoints.size()" << dstPoints.size() << std::endl;

            drawPoints2f(birdEyeViewWithPoints, dstPoints, cv::Scalar(0, 255, 255));
            drawLine(birdEyeViewWithPoints,leftLine,cv::Scalar(0, 255, 0));
            drawLine(birdEyeViewWithPoints,allMidpoints,cv::Scalar(255, 255, 255));
            drawLine(birdEyeViewWithPoints,rightLine,cv::Scalar(0, 0, 255));
            laptopTCP.sendFrame(birdEyeViewWithPoints);

            perspectiveChangeLineMinv(leftLine);
            perspectiveChangeLineMinv(rightLine);
            for (int i = 1; i < leftLine.size(); i++) {
                std::cout << "leftLine x: " << leftLine[i].x << " y:" << leftLine[i].y << std::endl;
            }
            for (int i = 1; i < rightLine.size(); i++)  {
                std::cout << "rightLine x: " << rightLine[i].x << " y:" << rightLine[i].y << std::endl;
            }

            allMidpoints = findMiddle(leftLine,rightLine,frameWidth,frameHeight);

            outputImage = cv::Mat::zeros(frame.size(),CV_8UC3);

            drawPoints(outputImage, leftLine, cv::Scalar(100, 100, 200));
            drawPoints(outputImage, rightLine, cv::Scalar(200, 200, 100));
            drawLine(outputImage,leftLine,cv::Scalar(0, 255, 0));
            drawLine(outputImage,allMidpoints,cv::Scalar(255, 255, 255));
            drawLine(outputImage,rightLine,cv::Scalar(0, 0, 255));
            laptopTCP.sendFrame(outputImage);
            
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            if (cv::waitKey(1) == 'q') {
                running = false;
                break;
            }
        }
    }
}


int main() {
    
    #if ENABLE_STREAMING == 1
        // Get the camera index from the shell script
        std::string cameraIndexStr = getCameraIndex();
        int cameraIndex = std::stoi(cameraIndexStr);

        cv::VideoCapture cap(cameraIndex);

        cap.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
        cap.set(cv::CAP_PROP_FPS, 60);

        if (!cap.isOpened()) {
            std::cerr << "Error: Could not open video device" << std::endl;
            return -1;
        }
    #endif
    #if ENABLE_READING_FROM_JPG == 1
        cv::VideoCapture cap;
    #endif

    // Start the capture and processing threads
    std::thread capture_thread(capture_frames, std::ref(cap));
    std::thread processing_thread(process_frame);

    // Wait for threads to finish
    capture_thread.join();
    processing_thread.join();

    // Release resources
    cap.release();
    cv::destroyAllWindows();

    return 0;
}