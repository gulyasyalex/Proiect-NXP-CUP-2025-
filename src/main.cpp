
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
            //std::string imagePath = "ogImage.jpg";
            //std::string imagePath = "ogCurves.jpg";
            //std::string imagePath = "ogCurves1.jpg";
            //std::string imagePath = "imagineSala1.jpg";
            //std::string imagePath = "imagineSala2.jpg";
            //std::string imagePath = "imagineSala3.jpg";
            //std::string imagePath = "imagineSala4.jpg";
            //std::string imagePath = "lines4.jpeg";
            //std::string imagePath = "lines5.jpeg";
            std::string imagePath = "imagine12022024_02.jpg";
            //std::string imagePath = "imagine12022024_03.jpg";

            cv::Mat frame = cv::imread(imagePath, cv::IMREAD_COLOR);
            
        //cv::Mat frame;
        //cap >> frame;
        if (frame.empty()) break;
        
        frame = resizeImage(frame, 320, 180);
        /*bool success = cv::imwrite("imagine12022024_03.jpg", frame);

        if (success) {
            std::cout << "Image successfully saved to " << std::endl;
        } else {
            std::cerr << "Failed to save the image!" << std::endl;
        }*/
        cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
        //frame = cropFrameTop(frame, 40);
        // Lock the mutex before updating the shared frame
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
        
        //fps = cap.get(cv::CAP_PROP_FPS);
        //std::cout << "Current FPS: " << fps << std::endl;
        //std::cout << "FPS(capture_frames): " << fps << std::endl;
    }
}

void process_frame() {
    
    double fps;
    int frame_count = 0;
    double time_start = cv::getTickCount();
    TcpConnection laptopTCP(9999);

    while (running) {
        cv::Mat frame;

        // Lock the mutex to safely access the latest frame
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

            //frame = cropFrameTop(frame,cutHeight);
            //cv::Mat edges = segmentEdges(frame);  
            cv::Mat edges;
            
            cv::threshold(frame, edges, 90, 255, cv::THRESH_BINARY); //fara LED 25
            cv::bitwise_not(edges, edges);
            //laptopTCP.sendFrame(edges);
            cv::Mat skeleton = skeletonizeFrame(edges);
            //laptopTCP.sendFrame(skeleton);

            //laptopTCP.sendFrame(labels);

            std::vector<std::vector<cv::Point>> lines = findLines(skeleton);

            cv::Mat outputImage = cv::Mat::zeros(frame.size(), CV_8UC3);  // Output image for drawing
      

            // Draw the original contour (blue)
            for (int i = 0; i < lines.size(); ++i) {
                for (int j = 0; j < lines[i].size() - 1; ++j) {
                    cv::line(outputImage, lines[i][j], lines[i][j + 1], cv::Scalar(255, 0, 0), 2);  // Blue
                }
            }
            
            laptopTCP.sendFrame(outputImage);

            for (int i = 0; i < lines.size(); i++){
                lines[i] = fitPolinomial(lines[i],false);

            }
            
            for (int i = 0; i < lines.size(); i++){
                
                lines[i] = evenlySpacePoints(lines[i], num_points);
            }

            for (int i = 0; i < lines.size(); ++i) {
                for (int j = 0; j < lines[i].size() - 1; ++j) {
                    cv::line(outputImage, lines[i][j], lines[i][j + 1], cv::Scalar(0, 0, 255), 2);  // Red
                }
            }
            
            laptopTCP.sendFrame(outputImage);
            for (int i = 0; i < lines.size(); i++) {
                
                lines[i] = removeHorizontalIf90Turn(frame,lines[i]);
            }
            getLeftRightLines(lines,leftLine,rightLine);
            
            /*cv::Point result;
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
            }*/
            

            
            std::vector<cv::Point> allMidpoints = findMiddle(leftLine,rightLine);


            outputImage = cv::Mat::zeros(frame.size(),CV_8UC3);
            for (int i = 1; i < allMidpoints.size(); i++) {
                cv::line(outputImage, allMidpoints[i-1], allMidpoints[i], cv::Scalar(255, 255, 255), 2);  // Green
            }
            for (int i = 1; i < lines[0].size(); i++) {
                cv::line(outputImage, lines[0][i-1], lines[0][i], cv::Scalar(0, 255, 0), 2);  // Green
            }
            
            for (int i = 1; i < lines[1].size(); i++) {
                cv::line(outputImage, lines[1][i-1], lines[1][i], cv::Scalar(0, 0, 255), 2);  // Green
            }
            cv::Mat output = perspectiveChange(outputImage);
            laptopTCP.sendFrame(output);
            //laptopTCP.sendFrame(outputImage);
            
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            if (cv::waitKey(1) == 'q') {
                running = false;
                break;
            }
        }
    }
}


int main() {
    
    /*// Get the camera index from the shell script
    std::string cameraIndexStr = getCameraIndex();
    int cameraIndex = std::stoi(cameraIndexStr);

    cv::VideoCapture cap(cameraIndex);

    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
    cap.set(cv::CAP_PROP_FPS, 60);

    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open video device" << std::endl;
        return -1;
    }*/
   cv::VideoCapture cap;

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