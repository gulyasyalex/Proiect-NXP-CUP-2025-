
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <cmath>
#include "config.h"
#include "CameraFunctions/segmentation.h"
#include "CameraFunctions/line_detection.h"
#include "CameraFunctions/camera_setup.h"
#include "PurePursuit/pure_pursuit.h"
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
std::vector<cv::Point2f> leftLine;
std::vector<cv::Point2f> rightLine;

void capture_frames(cv::VideoCapture& cap) {
    
    double fps;
    int frame_count = 0;
    double time_start = cv::getTickCount();

    while (running) {
        try{
            #if 1 == ENABLE_STREAMING
                cv::Mat frame;
                cap >> frame;
            
            #else
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
                //std::string imagePath = "imagineThreshold_01.jpg";
                //std::string imagePath = "imagineThreshold_02.jpg";
                //std::string imagePath = "imagineThreshold_03.jpg";
                //std::string imagePath = "imagineThreshold_04.jpg";
                //std::string imagePath = "imagine08122024_01.jpg";
                //std::string imagePath = "imagine08122024_02.jpg";
                //std::string imagePath = "imagine08122024_03.jpg";
                //std::string imagePath = "imagine08122024_05.jpg";
                //std::string imagePath = "imagine08122024_06.jpg";
                //std::string imagePath = "imagine08122024_07.jpg";
                //std::string imagePath = "imagine08122024_08.jpg";

                cv::Mat frame = cv::imread(imagePath, cv::IMREAD_COLOR);
            #endif
            
            if (frame.empty()) break;
            
            frame = resizeImage(frame, 320, 180);
            //saveImage("imagine09122024_01.jpg",frame);
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
        catch (const std::exception& e) {
            std::cerr << "Error: " << e.what() << "\n";
        }
    }
}

void process_frame() {
    
    double fps;
    int frame_count = 0;
    double time_start = cv::getTickCount();
    TcpConnection laptopTCP(9999);
    
    initPerspectiveVariables();
    while (running) {
        try{
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

                std::vector<std::vector<cv::Point2f>> lines = findLines(skeleton);

                cv::Mat outputImage = cv::Mat::zeros(frame.size(), CV_8UC3);  // Output image for drawing
        
                drawLines(outputImage,lines,cv::Scalar(255, 0, 0));
                /*for (const auto& line : lines){
                    //drawPoints(outputImage, line, cv::Scalar(255, 0, 0));
                    for (const auto& point : line){
                        std::cout << "Point.x:" << point.x << " Point.y:" << point.y << std::endl;
                    }
                }*/
                //laptopTCP.sendFrame(outputImage);

                for (int i = 0; i < lines.size(); i++){
                    lines[i] = fitPolinomial(lines[i],false);
                }
                drawLines(outputImage,lines,cv::Scalar(0, 0, 255));
                laptopTCP.sendFrame(outputImage);

                #if 1 != ENABLE_CALIBRATE_CAMERA 
                bool has90degreeTurn = false;
                for (int i = 0; i < lines.size(); i++){
                    lines[i] = perspectiveChangeLine(lines[i], MatrixBirdsEyeView);
                    has90degreeTurn = removeHorizontalIf90Turn(lines[i]);
                    extendLineToEdges(lines[i], widthBirdsEyeView, heightBirdsEyeView);
                    lines[i] = evenlySpacePoints(lines[i], num_points);
                    if (has90degreeTurn) {
                        std::vector<cv::Point2f> temp;
                        temp = lines[i]; 
                        lines.clear();
                        lines.emplace_back(temp); 
                    }

                }
                #endif

                getLeftRightLines(lines,leftLine,rightLine);
                std::vector<cv::Point2f> allMidpoints = findMiddle(leftLine,rightLine,widthBirdsEyeView,heightBirdsEyeView);
                
                #if 1 == ENABLE_CALIBRATE_CAMERA 
                    // TO CALIBRATE CAMERA
                    cv::Point2f result;
                    std::vector<cv::Point2f> interpolatedPoints;
                        extendLineToEdges(leftLine, frameWidth, frameHeight);
                        extendLineToEdges(rightLine, frameWidth, frameHeight);
                        interpolatedPoints.push_back(interpolateClosestPoints(leftLine, 45));
                        interpolatedPoints.push_back(interpolateClosestPoints(leftLine, 170));
                        interpolatedPoints.push_back(interpolateClosestPoints(rightLine, 45));
                        interpolatedPoints.push_back(interpolateClosestPoints(rightLine, 170));
                        interpolatedPoints.push_back(interpolateClosestPoints(allMidpoints, frameHeight - 5));
                        // Write interpolated points to a TXT file
                        writePointsToTxt(interpolatedPoints, "interpolated_points.txt");
                        initPerspectiveVariables();
                        laptopTCP.sendFrame(outputImage);

                #else    

                radiusIncrease(lookAheadDistance);                
                pointMoveAcrossFrame(carInFramePositionBirdsEye);
                double tempLookAheadDistance = shortestDistanceToCurve(allMidpoints, carInFramePositionBirdsEye, lookAheadDistance);
               
                // Create a frame of the desired size
                cv::Size frameSize(widthBirdsEyeView, heightBirdsEyeView);

                // Initialize an empty image (black by default)
                cv::Mat birdEyeViewWithPoints = cv::Mat::zeros(frameSize, CV_8UC3); // 3 channels (color)

                // Find intersections
                cv::Point2f lookAheadPoint = findHighestIntersection(allMidpoints, carInFramePositionBirdsEye, tempLookAheadDistance);
             
                cv::circle(birdEyeViewWithPoints, lookAheadPoint, 5, cv::Scalar(254, 34, 169), -1);
                drawCircle(birdEyeViewWithPoints, carInFramePositionBirdsEye, tempLookAheadDistance, cv::Scalar(254, 34, 169));
                cv::circle(birdEyeViewWithPoints, carInFramePositionBirdsEye, 5, cv::Scalar(254, 34, 169), -1);
                drawPoints2f(birdEyeViewWithPoints, dstPoints, cv::Scalar(0, 255, 255));
                drawLine(birdEyeViewWithPoints,leftLine,cv::Scalar(0, 255, 0));
                drawLine(birdEyeViewWithPoints,allMidpoints,cv::Scalar(255, 255, 255));
                drawLine(birdEyeViewWithPoints,rightLine,cv::Scalar(0, 0, 255));
                laptopTCP.sendFrame(birdEyeViewWithPoints);

               


                leftLine = perspectiveChangeLine(leftLine, MatrixInverseBirdsEyeView);
                rightLine = perspectiveChangeLine(rightLine, MatrixInverseBirdsEyeView);
                allMidpoints = perspectiveChangeLine(allMidpoints, MatrixInverseBirdsEyeView);
                lookAheadPoint = perspectiveChangePoint(lookAheadPoint, MatrixInverseBirdsEyeView);

                #endif
                outputImage = cv::Mat::zeros(frame.size(),CV_8UC3);

                drawPoints2f(outputImage, srcPoints, cv::Scalar(0, 255, 255));
                std::vector<cv::Point2f> horizontalLine;
                horizontalLine.push_back(srcPoints[1]);
                horizontalLine.push_back(srcPoints[3]);
                
                cv::circle(outputImage, lookAheadPoint, 5, cv::Scalar(254, 34, 169), -1);
                cv::circle(outputImage, carInFramePosition, 5, cv::Scalar(254, 34, 169), -1);
                drawLine(outputImage,horizontalLine,cv::Scalar(0, 255, 255));
                #if 1 != ENABLE_CALIBRATE_CAMERA 
                drawPoints(outputImage, leftLine, cv::Scalar(0, 255, 0));
                drawPoints(outputImage, rightLine, cv::Scalar(0, 0, 255));
                drawLine(outputImage,leftLine,cv::Scalar(0, 255, 0));
                drawLine(outputImage,allMidpoints,cv::Scalar(255, 255, 255));
                drawLine(outputImage,rightLine,cv::Scalar(0, 0, 255));
                #endif

                //laptopTCP.sendFrame(outputImage);
                cv::cvtColor(frame, frame, cv::COLOR_GRAY2BGR);

                // Add the visualization (outputImage) on top of the original frame
                cv::Mat overlayedImage;
                cv::addWeighted(frame, 1.0, outputImage, 1.0, 0, overlayedImage);
                laptopTCP.sendFrame(overlayedImage);
            
                //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                if (cv::waitKey(1) == 'q') {
                    running = false;
                    break;
                }
            }
        }
        catch (const std::exception& e) {
            std::cerr << "Error: " << e.what() << "\n";
        }
    }
}


int main() {
    
    #if 1 == ENABLE_STREAMING
        // Get the camera index from the shell script
        std::string cameraIndexStr = getCameraIndex();
        int cameraIndex = std::stoi(cameraIndexStr);
        std::cout << "cameraIndex:" << cameraIndex << std::endl;
        cv::VideoCapture cap(cameraIndex);

        cap.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
        cap.set(cv::CAP_PROP_FPS, 60);

        if (!cap.isOpened()) {
            std::cerr << "Error: Could not open video device" << std::endl;
            return -1;
        }
    #else
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