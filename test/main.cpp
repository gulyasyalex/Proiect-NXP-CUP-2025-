
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <cmath>
#include "config.h"
#include "CameraFunctions/segmentation.h"
#include "CameraFunctions/line_detection.h"
#include "tcp_connect.h"
#include "libpixyusb2.h"
//#include "include/camera_setup.h"
//#include "include/color_segmentation.h"

#include <thread>
#include <atomic>
#include <mutex>

std::atomic<bool> running(true); // Flag to control the capture loop
cv::Mat latest_frame; // Shared frame variable
std::mutex frame_mutex; // Mutex for synchronizing access to the frame

Pixy2 pixy;

void capture_frames(cv::VideoCapture& cap) {
    
    while (running) {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) break;
        
        frame = resizeImage(frame, 320, 180);

        // Lock the mutex before updating the shared frame
        {
            std::lock_guard<std::mutex> lock(frame_mutex);
            latest_frame = frame.clone(); // Update the latest frame
        }
    }
}

void process_frame() {
    
    double fps;
    int frame_count = 0;
    double time_start = cv::getTickCount();
    TcpConnection laptopTCP(9999);
    //cv::namedWindow("Camera - 640x360 MJPEG", cv::WINDOW_AUTOSIZE);
    while (running) {
        cv::Mat frame;

        // Lock the mutex to safely access the latest frame
        {
            std::lock_guard<std::mutex> lock(frame_mutex);
            frame = latest_frame.clone(); // Always copy the latest frame
        }
        

        // Process the frame if it is not empty (mainly for the first iterations)
        if (!frame.empty()) {

            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

            // Calculate FPS
            frame_count++;
            double time_elapsed = (cv::getTickCount() - time_start) / cv::getTickFrequency();
            if (time_elapsed >= 1.0) {  // Update FPS every second
                fps = frame_count / time_elapsed;
                frame_count = 0;
                time_start = cv::getTickCount();
            }

            std::cout << "FPS: " << fps << std::endl;

            /*// Display the FPS on the frame
            std::string fps_text = "FPS: " + std::to_string(fps);
            cv::putText(frame, fps_text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);

            // Display the frame
            cv::imshow("Camera - 640x360 MJPEG", frame);
            */
            //frame = cropFrameTop(frame,cutHeight);
            cv::Mat edges = segmentEdges(frame);  

            laptopTCP.sendFrame(frame);
            std::vector<std::vector<cv::Point>> lines = findLines(edges);

            cv::Mat outputImage = cv::Mat::zeros(frame.size(), CV_8UC3);  // Output image for drawing
        
        

            //std::cout << "lines.size(): " << lines.size() << std::endl;
            // Draw the original contour (blue)
            for (int i = 0; i < lines.size(); ++i) {
                for (int j = 0; j < lines[i].size() - 1; ++j) {
                    cv::line(outputImage, lines[i][j], lines[i][j + 1], cv::Scalar(255, 0, 0), 2);  // Blue
                }
            }

            for (int i = 0; i < lines.size(); i++){
                lines[i] = fitPolinomial(lines[i],false);

            }
            // Draw the original contour (red)
            for (int i = 0; i < lines.size(); ++i) {
                for (int j = 0; j < lines[i].size() - 1; ++j) {
                    cv::line(outputImage, lines[i][j], lines[i][j + 1], cv::Scalar(0, 0, 255), 2);  // Red
                }
            }
            laptopTCP.sendFrame(outputImage);
            // Display the result
            //cv::imshow("OG Curves", outputImage); 
            
            
            /*for (int i = 0; i < lines.size(); i++){
                
                lines[i] = evenlySpacePoints(lines[i], num_points);
            }
            for (int i = 0; i < lines.size(); i++) {
                
                lines[i] = removeHorizontalIf90Turn(frame,lines[i]);
            }
            std::vector<cv::Point> allMidpoints = findMiddle(edges,lines);


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

            
        
            outputImage = resizeImage(outputImage, frameWidth/1.6, frameHeight/1.6);
            // Display the image with lines colored
            cv::imshow("middle Lines", outputImage);
            */
            //cv::imshow("edges", edges);
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            if (cv::waitKey(1) == 'q') {
                running = false;
                break;
            }
        }
    }
}


int main() {
    cv::VideoCapture cap(3);
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH, frameWidth);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, frameHeight);
    //cap.set(cv::CAP_PROP_FPS, 50);

    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open video device" << std::endl;
        return -1;
    }

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


int main() {
    //std::string imagePath = "ogImage.jpg";
    //std::string imagePath = "ogCurves.jpg";
    //std::string imagePath = "ogCurves1.jpg";
    //std::string imagePath = "imagineSala1.jpg";
    //std::string imagePath = "imagineSala2.jpg";
    //std::string imagePath = "imagineSala3.jpg";
    //std::string imagePath = "imagineSala4.jpg";
    //std::string imagePath = "lines4.jpeg";
    //std::string imagePath = "lines5.jpeg";

    //cv::Mat frame = cv::imread(imagePath, cv::IMREAD_COLOR);

    //frame = resizeImage(frame, frameWidth, frameHeight);
    
    cv::VideoCapture cap(2); // Use index 2 for /dev/video2

    //cv::VideoCapture cap("video.mp4");
    if(!cap.isOpened()){
        std::cout << "Error opening video stream or file" << std::endl;
        return -1;
    }

    // Set the format to MJPEG and the resolution to 640x360
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 360);
    //cap.set(cv::CAP_PROP_FPS, 120); // Set to 120 FPS for 640x360

    // Create a window to display the video feed
    cv::namedWindow("Camera - 640x360 MJPEG", cv::WINDOW_AUTOSIZE);

    double fps;
    int frame_count = 0;
    double time_start = cv::getTickCount();

    //TcpConnection laptopTCP(9999);

    while(true){
        cv::Mat frame;
        // Capture frame by frame
        cap >> frame; 
        

        // If the frame is empty, break imediately
        if (frame.empty())
            break;

        // Calculate FPS
        frame_count++;
        double time_elapsed = (cv::getTickCount() - time_start) / cv::getTickFrequency();
        if (time_elapsed >= 1.0) {  // Update FPS every second
            fps = frame_count / time_elapsed;
            frame_count = 0;
            time_start = cv::getTickCount();
        }

        // Display the FPS on the frame
        std::string fps_text = "FPS: " + std::to_string(fps);
        cv::putText(frame, fps_text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);

        // Display the frame
        cv::imshow("Camera - 640x360 MJPEG", frame);

        //frame = cropFrameTop(frame,cutHeight);
        cv::Mat edges = segmentEdges(frame);  

        std::vector<std::vector<cv::Point>> lines = findLines(edges);

        cv::Mat outputImage = cv::Mat::zeros(frame.size(), CV_8UC3);  // Output image for drawing
    
        // Draw the original contour (blue)
        for (int i = 0; i < lines.size(); ++i) {
            for (int j = 0; j < lines[i].size() - 1; ++j) {
                cv::line(outputImage, lines[i][j], lines[i][j + 1], cv::Scalar(255, 0, 0), 2);  // Blue
            }
        }

        for (int i = 0; i < lines.size(); i++){
            lines[i] = fitPolinomial(lines[i],false);

        }
        // Draw the original contour (red)
        for (int i = 0; i < lines.size(); ++i) {
            for (int j = 0; j < lines[i].size() - 1; ++j) {
                cv::line(outputImage, lines[i][j], lines[i][j + 1], cv::Scalar(0, 0, 255), 2);  // Red
            }
        }
        outputImage = resizeImage(outputImage, frameWidth/1.6, frameHeight/1.6);
        // Display the result
        cv::imshow("OG Curves", outputImage); 
        
        //laptopTCP.sendFrame(outputImage);
        for (int i = 0; i < lines.size(); i++){
            
            lines[i] = evenlySpacePoints(lines[i], num_points);
        }
        for (int i = 0; i < lines.size(); i++) {
            
            lines[i] = removeHorizontalIf90Turn(frame,lines[i]);
        }
        std::vector<cv::Point> allMidpoints = findMiddle(edges,lines);


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

        
    
        outputImage = resizeImage(outputImage, frameWidth/1.6, frameHeight/1.6);
        // Display the image with lines colored
        cv::imshow("middle Lines", outputImage);
        //laptopTCP.sendFrame(outputImage);
        //cv::Mat perspective = perspectiveChange(edges, static_cast<float>(cutHeight));
        //cv::imshow("Edges Perspective",perspective);

        char c = (char)cv::waitKey(25);
        if(c == 27)
        break;
    }
    // Release the camera and destroy all windows
    cap.release();
    cv::destroyAllWindows();
    cv::waitKey(0);
    return 0;
}



/*
int main() {
    // Open the video device at /dev/video2
    cv::VideoCapture cap(2); // Use index 2 for /dev/video2

    // Check if the device opened successfully
    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open video device" << std::endl;
        return -1;
    }

    // Set the format to MJPEG and the resolution to 1280x720
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 360);
    //cap.set(cv::CAP_PROP_FPS, 120); // Set to 120 FPS for 1280x720


    // Create a window to display the video feed
    cv::namedWindow("Camera - 640x360 MJPEG", cv::WINDOW_AUTOSIZE);

    double fps;
    int frame_count = 0;
    double time_start = cv::getTickCount();

    TcpConnection laptopTCP(9999);

    while (true) {
        cv::Mat frame;

        // Capture a new frame
        cap >> frame;

        // Check if the frame was captured successfully
        if (frame.empty()) {
            std::cerr << "Error: Could not capture frame" << std::endl;
            break;
        }

        // Calculate FPS
        frame_count++;
        double time_elapsed = (cv::getTickCount() - time_start) / cv::getTickFrequency();
        if (time_elapsed >= 1.0) {  // Update FPS every second
            fps = frame_count / time_elapsed;
            frame_count = 0;
            time_start = cv::getTickCount();
        }

        // Display the FPS on the frame
        std::string fps_text = "FPS: " + std::to_string(fps);
        cv::putText(frame, fps_text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);

        // Display the frame
        cv::imshow("Camera - 640x360 MJPEG", frame);
        
        laptopTCP.sendFrame(frame);

        // Break the loop on 'q' key press
        if (cv::waitKey(1) == 'q') {
            break;
        }
    }

    // Release the camera and destroy all windows
    cap.release();
    cv::destroyAllWindows();

    return 0;
}
*/