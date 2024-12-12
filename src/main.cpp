
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <thread>
#include <atomic>
#include <mutex>
#include <cmath>
#include "config.h"
#include "CameraFunctions/segmentation.h"
#include "CameraFunctions/line_detection.h"
#include "CameraFunctions/camera_setup.h"
#include "PurePursuit/pure_pursuit.h"
#include "tcp_connect.h"
//#include "include/camera_setup.h"
//#include "include/color_segmentation.h"


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
                for (int i = 0; i < lines.size(); i++){
                    if(isIntersection && lines[i][0].y >= frameHeight * lineStartPointY){
                        isPastIntersection = true;
                        isIntersection = false;
                    }
                    if(!isIntersection){
                        lines[i] = perspectiveChangeLine(lines[i], MatrixBirdsEyeView);
                        isIntersection = removeHorizontalIf90Turn(lines[i]);
                        extendLineToEdges(lines[i], birdsEyeViewWidth, birdsEyeViewHeight);
                        lines[i] = evenlySpacePoints(lines[i], num_points);
                        if (isIntersection) {
                            std::vector<cv::Point2f> temp;
                            temp = lines[i]; 
                            lines.clear();
                            lines.emplace_back(temp); 
                            break;
                        }
                    }

                }
                #endif

                /*
                    If car finds intersection it looks for the middle line 
                    and keeps middle till it goes past intersection.
                    Assigns middle line only once!
                */
               
                if (isPastIntersection){
                    getLeftRightLines(lines,leftLine,rightLine);
                    allMidPoints = findMiddle(leftLine,rightLine,birdsEyeViewWidth,birdsEyeViewHeight);
                    std::cout << "Was here\n";
                    // Used to handle the case when no point is found in the middle of the intersection
                    if (isIntersection)
                            isPastIntersection = false;
                    
                }
                std::cout << "allMidPoints.size(): "<< allMidPoints.size() << "\n";
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
                        interpolatedPoints.push_back(interpolateClosestPoints(allMidPoints, frameHeight - 5));
                        // Write interpolated points to a TXT file
                        writePointsToTxt(interpolatedPoints, "interpolated_points.txt");
                        initPerspectiveVariables();
                        laptopTCP.sendFrame(outputImage);

                #else    
                
                //radiusIncrease(lookAheadDistance);                
                //pointMoveAcrossFrame(carInFramePositionBirdsEye, carTopPoint);
                double tempLookAheadDistance = shortestDistanceToCurve(allMidPoints, carInFramePositionBirdsEye, lookAheadDistance);
               
                // Create a frame of the desired size
                cv::Size frameSize(birdsEyeViewWidth, birdsEyeViewHeight);

                // Initialize an empty image (black by default)
                cv::Mat birdEyeViewWithPoints = cv::Mat::zeros(frameSize, CV_8UC3); // 3 channels (color)

                // Find intersections
                cv::Point2f lookAheadPoint = findHighestIntersection(allMidPoints, carInFramePositionBirdsEye, tempLookAheadDistance);
             
                cv::line(birdEyeViewWithPoints, carInFramePositionBirdsEye, lookAheadPoint, cv::Scalar(123, 10, 255), 2); 
                cv::line(birdEyeViewWithPoints, carInFramePositionBirdsEye, carTopPoint, cv::Scalar(123, 10, 255), 2);  
                
                cv::circle(birdEyeViewWithPoints, lookAheadPoint, 5, cv::Scalar(254, 34, 169), -1);
                drawCircle(birdEyeViewWithPoints, carInFramePositionBirdsEye, tempLookAheadDistance, cv::Scalar(254, 34, 169));
                cv::circle(birdEyeViewWithPoints, carInFramePositionBirdsEye, 5, cv::Scalar(254, 34, 169), -1);
                drawPoints2f(birdEyeViewWithPoints, dstPoints, cv::Scalar(0, 255, 255));
                drawLineVector(birdEyeViewWithPoints,leftLine,cv::Scalar(0, 255, 0));
                drawLineVector(birdEyeViewWithPoints,allMidPoints,cv::Scalar(255, 255, 255));
                drawLineVector(birdEyeViewWithPoints,rightLine,cv::Scalar(0, 0, 255));
                laptopTCP.sendFrame(birdEyeViewWithPoints);

                std::vector<cv::Point2f> l_leftLine = perspectiveChangeLine(leftLine, MatrixInverseBirdsEyeView);
                std::vector<cv::Point2f> l_rightLine = perspectiveChangeLine(rightLine, MatrixInverseBirdsEyeView);
                std::vector<cv::Point2f> l_allMidPoints = perspectiveChangeLine(allMidPoints, MatrixInverseBirdsEyeView);
                cv::Point2f l_lookAheadPoint = perspectiveChangePoint(lookAheadPoint, MatrixInverseBirdsEyeView);

                #endif
                outputImage = cv::Mat::zeros(frame.size(),CV_8UC3);

                drawPoints2f(outputImage, srcPoints, cv::Scalar(0, 255, 255));
                std::vector<cv::Point2f> horizontalLine;
                horizontalLine.push_back(srcPoints[1]);
                horizontalLine.push_back(srcPoints[3]);
                
                cv::circle(outputImage, l_lookAheadPoint, 5, cv::Scalar(254, 34, 169), -1);
                cv::circle(outputImage, carInFramePosition, 5, cv::Scalar(254, 34, 169), -1);
                drawLineVector(outputImage,horizontalLine,cv::Scalar(0, 255, 255));
                #if 1 != ENABLE_CALIBRATE_CAMERA 
                drawPoints(outputImage, l_leftLine, cv::Scalar(0, 255, 0));
                drawPoints(outputImage, l_rightLine, cv::Scalar(0, 0, 255));
                drawLineVector(outputImage,l_leftLine,cv::Scalar(0, 255, 0));
                drawLineVector(outputImage,l_allMidPoints,cv::Scalar(255, 255, 255));
                drawLineVector(outputImage,l_rightLine,cv::Scalar(0, 0, 255));
                #endif

                //laptopTCP.sendFrame(outputImage);
                cv::cvtColor(frame, frame, cv::COLOR_GRAY2BGR);

                // Add the visualization (outputImage) on top of the original frame
                cv::Mat overlayedImage;
                cv::addWeighted(frame, 1.0, outputImage, 1.0, 0, overlayedImage);
                laptopTCP.sendFrame(overlayedImage);

                double angle = calculateSignedAngle(carTopPoint, carInFramePositionBirdsEye, lookAheadPoint);
                std::cout << "Steering Angle: " << angle << std::endl;
            
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