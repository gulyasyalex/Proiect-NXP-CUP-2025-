#ifndef CAMERA_PROCESSING_HPP
#define CAMERA_PROCESSING_HPP

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <atomic>
#include <thread>
#include <mutex>
#include <vector>
#include <cmath>
#include <fstream>
#include <queue>
#include <unistd.h>
#include "config.h"
#include "mathFunctions.h"
#include "thinning.h"
#include "TcpConnection.hpp"
#include "PurePursuit/purePursuit.h"
#include <cstdlib> // Required for exit()

class CameraProcessing {
    private:
        int frameWidth = captureFrameWidth;
        int frameHeight = captureFrameHeight;
        cv::VideoCapture cap;
        std::thread captureThread;
        std::thread processingThread;
        cv::Mat latestFrame;
        std::mutex frameMutex;

        double fpsCaptureFrames;
        double fpsProcessFrames;

        std::vector<cv::Point2f> leftLine;
        std::vector<cv::Point2f> rightLine;
        std::vector<cv::Point2f> allMidPoints;
        std::vector<cv::Point2f> srcPoints;
        std::vector<cv::Point2f> dstPoints;
        cv::Mat MatrixBirdsEyeView;
        cv::Mat MatrixInverseBirdsEyeView;
   
        cv::Point2f carInFramePosition;
        cv::Point2f carInFramePositionBirdsEye;
        cv::Point2f carTopPoint;
        double trackLaneWidthInPixel;
        TcpConnection laptopTCP{9999};

        double lookAheadDistance = 150.0f;  // It's the radius of the circle starting from carInFramePositionBirdsEye
        bool isIntersection = false;    // Used to check if we have an intersection on the racetrack, a crosspath
        bool isPastIntersection = true;    // Used to keep same middle line till we cross intersection
        double lineStartPointY = 0.8;       // birdsEyeViewHeight * lineStartPointY = Y threshold for isPastIntersection


        // Used in getLeftRightLines()
        cv::Point2f firstPointLeftLine = undefinedPoint;
        cv::Point2f firstPointRightLine = undefinedPoint;
        cv::Point2f firstPointSingleLine = undefinedPoint;

    public:
        std::atomic<bool> running{false};
        // Default constructor
        CameraProcessing();
        // Constructor with camera parameters
        CameraProcessing(int cameraIndex, int width, int height, int fps);
        // Destructor stops capturing
        ~CameraProcessing();
        // Set camera parameters
        void setParameters(int cameraIndex, int width, int height, int fps);
        // Start capturing frames
        void startCapture(); 
        // Start capturing frames
        void startFrameProcessing();
        bool isRunning() const;
        // Stop capturing frames
        void stopCapture();
        void stopFrameProcessing();
        // Capture frames from the camera
        void captureFrames();
        // Process frames
        void processFrames();
        // Retrieve the latest frame for processing
        cv::Mat getLatestFrame();
        // Resize the image to the specified dimensions
        cv::Mat resizeImage(const cv::Mat& frame, int providedFrameWidth, int providedFrameHeight);
        // Cuts pixel rows from image's top part
        cv::Mat cropFrameTop(const cv::Mat& frame, double l_topCutOffPercentage);
        cv::Mat skeletonizeFrame(cv::Mat& thresholdedImage);
        // Apply color segmentation to isolate specific features in the image
        cv::Mat segmentEdges(const cv::Mat& frame);
        // Function to find lines in a skeletonized frame
        std::vector<std::vector<cv::Point2f>> findLines(const cv::Mat& thresholdedImage);        
        int customConnectedComponentsWithThreshold(const cv::Mat& binaryImage, cv::Mat& labelImage, int radius, std::vector<std::vector<cv::Point2f>>& lines);
        // Function to smooth a vector of points using a moving average filter used in fitPolinomial
        std::vector<cv::Point2f> smoothPoints(const std::vector<cv::Point2f>& points, int windowSize);
        // Function to fit a polynomial to a set of points and return interpolated points
        std::vector<cv::Point2f> fitPolinomial(const std::vector<cv::Point2f>& line, bool isMiddleLine);
        void extendLineToEdges(std::vector<cv::Point2f>& line, int providedFrameWidth, int providedFrameHeight);
        // Function to evenly space points along the curve based on arc length
        std::vector<cv::Point2f> evenlySpacePoints(const std::vector<cv::Point2f>& line, int num_points);
        bool are2PointsHorizontal(const cv::Point2f& p1, const cv::Point2f& p2, double slopeThreshold = 1);
        // Function to remove horizontal sections if a 90-degree turn is detected
        bool removeHorizontalIf90Turn(std::vector<cv::Point2f>& line);
        // Function that handles 2 Lines, 1 Line and No line cases
        void getLeftRightLines(const std::vector<std::vector<cv::Point2f>>& lines, std::vector<cv::Point2f>& leftFitted, std::vector<cv::Point2f>& rightFitted); 
        std::vector<cv::Point2f> findMiddle(std::vector<cv::Point2f>& leftLine, std::vector<cv::Point2f>& rightLine, int providedFrameWidth, int providedFrameHeight);
        cv::Point2f interpolateClosestPoints(const std::vector<cv::Point2f>& points, int targetY);
        // Perspective transform related methods
        void initPerspectiveVariables();
        cv::Point2f perspectiveChangePoint(const cv::Point2f& point, const cv::Mat& transformMatrix);
        std::vector<cv::Point2f> perspectiveChangeLine(const std::vector<cv::Point2f>& line, const cv::Mat& transformMatrix);
        void saveImage(const std::string& filename, const cv::Mat& frame);
        void writePointsToTxt(const std::vector<cv::Point2f>& points, const std::string& filename);
        std::vector<cv::Point2f> readPointsFromTxt(const std::string& filename);
        // Function to draw the points on the frame
        void drawPoints(cv::Mat& frame, const std::vector<cv::Point2f>& points, const cv::Scalar& color);
        // Function to draw a horizontal line from a height point on the frame
        void drawHorizontalFromHeight(cv::Mat& frame, const int heightPoint, const cv::Scalar& color);
        // Function to draw the line on the frame
        void drawLineVector(cv::Mat& frame, const std::vector<cv::Point2f>& line, const cv::Scalar& color);
        // Function to draw the line on the frame
        void drawLines(cv::Mat& frame, const std::vector<std::vector<cv::Point2f>> lines, const cv::Scalar& color);
        void drawCircle(cv::Mat& image, const cv::Point2f& center, int radius, const cv::Scalar& color, int thickness = 1);
};

// Default constructor
CameraProcessing::CameraProcessing() {}

// Constructor with camera parameters
CameraProcessing::CameraProcessing(int cameraIndex, int width, int height, int fps)
{
    setParameters(cameraIndex, width, height, fps);
}

// Destructor stops capturing
CameraProcessing::~CameraProcessing() {
    stopCapture();
}

// Set camera parameters
void CameraProcessing::setParameters(int cameraIndex, int width, int height, int fps) {
    cap.open(cameraIndex, cv::CAP_V4L2);
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    cap.set(cv::CAP_PROP_FPS, fps);

    double actualWidth = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    double actualHeight = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    double actualFPS = cap.get(cv::CAP_PROP_FPS);

    std::cout << "Camera settings:" << std::endl;
    std::cout << "  Frame width: " << actualWidth << std::endl;
    std::cout << "  Frame height: " << actualHeight << std::endl;
    std::cout << "  FPS: " << actualFPS << std::endl;

    if (!cap.isOpened()) {
        throw std::runtime_error("Error: Could not open video device");
    }
}

// Start capturing frames
void CameraProcessing::startCapture() {
    this->running = true;
    this->captureThread = std::thread(&CameraProcessing::captureFrames, this);
} 

// Start capturing frames
void CameraProcessing::startFrameProcessing() {
    this->running = true;
    this->processingThread = std::thread(&CameraProcessing::processFrames, this);
}

bool CameraProcessing::isRunning() const {
    return this->running.load();
}

// Stop capturing frames
void CameraProcessing::stopCapture() {
    if (this->running) {
        this->running = false;
        if (this->captureThread.joinable()) {
            this->captureThread.join();
        }
    }
}

void CameraProcessing::stopFrameProcessing() {
    if (this->running) {
        this->running = false;
        if (this->processingThread.joinable()) {
            this->processingThread.join();
        }
    }
}

// Capture frames from the camera
void CameraProcessing::captureFrames() {
        
    double fps;
    int frameCount = 0;
    double timeStart = cv::getTickCount();

    while (this->running) {
        try{
            #if 1 == ENABLE_CAMERA_STREAMING
                cv::Mat frame;
                this->cap >> frame;
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

            if (frame.empty()) {
                std::cerr << "Error: Empty frame captured" << std::endl;
                break;
            }

            frame = resizeImage(frame, resizeFrameWidth, resizeFrameHeight);
            //saveImage("imagine09122024_01.jpg",frame);
            cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
            // TO CROP
            frame = cropFrameTop(frame, topCutOffPercentage);

            {
                std::lock_guard<std::mutex> lock(frameMutex);
                this->latestFrame = frame.clone();
            }

            frameCount++;
            double time_elapsed = (cv::getTickCount() - timeStart) / cv::getTickFrequency();
            if (time_elapsed >= 1.0) {  // Update FPS every second
                fps = frameCount / time_elapsed;
                frameCount = 0;
                timeStart = cv::getTickCount();
            }
            this->fpsCaptureFrames = fps;
        }
        catch (std::exception& e)
        {
            std::cerr << "Error: " << e.what() << "\n";
        }
    }
}
        
// Process frames
void CameraProcessing::processFrames() {
    
    double fps;
    int frameCount = 0;
    double timeStart = cv::getTickCount();

    this->initPerspectiveVariables();
    while (this->running) {
        try {
            cv::Mat frame;

            {
                std::lock_guard<std::mutex> lock(frameMutex);
                frame = this->latestFrame.clone(); // Always copy the latest frame
            }

            if (!frame.empty()) {
                // Perform your frame processing here
                frameCount++;
                double time_elapsed = (cv::getTickCount() - timeStart) / cv::getTickFrequency();
                if (time_elapsed >= 1.0) {
                    fps = frameCount / time_elapsed;
                    frameCount = 0;
                    timeStart = cv::getTickCount();
                }

                std::cout << "FPS(processFrames): " << fps << " FPS(captureFrames): " << this->fpsCaptureFrames << std::endl;
                //serial.writeToSerial("FPS(processFrames): " + std::to_string(fps) + 
                //     " FPS(captureFrames): " + std::to_string(this->fpsCaptureFrames));

                cv::TickMeter timer;
                // * timer.start();
                cv::Mat thresholdFrame = this->segmentEdges(frame);
                // * timer.stop();
                // * std::cout << "segmentEdges Time: " << timer.getTimeMilli() << " ms" << std::endl;
                #if 1 == ENABLE_TCP_OUTPUT 
                    laptopTCP.sendFrame(thresholdFrame);
                #endif
                #if 1 != ENABLE_CAMERA_THRESHOLD_CHECK
                    // * timer.start();
                    cv::Mat skeleton = this->skeletonizeFrame(thresholdFrame); 
                    // * timer.stop();
                    // * std::cout << "skeletonizeFrame Time: " << timer.getTimeMilli() << " ms" << std::endl;


                    // * timer.start();
                    std::vector<std::vector<cv::Point2f>> lines = this->findLines(skeleton);
                    // * timer.stop();
                    // * std::cout << "findLines Time: " << timer.getTimeMilli() << " ms" << std::endl;

                    cv::Mat outputImage = cv::Mat::zeros(frame.size(), CV_8UC3);  // Output image for drawing
                    
                    #if 1 == ENABLE_TCP_OUTPUT 
                        this->drawLines(outputImage,lines,cv::Scalar(255, 0, 0));
                        /*for (const auto& line : lines){
                            //drawPoints(outputImage, line, cv::Scalar(255, 0, 0));
                            for (const auto& point : line){
                                std::cout << "Point.x:" << point.x << " Point.y:" << point.y << std::endl;
                            }
                        }*/
                        //laptopTCP.sendFrame(outputImage);
                    #endif
                    
                    // * timer.start();
                    for (int i = 0; i < lines.size(); i++){  
                        lines[i] = this->fitPolinomial(lines[i],false);
                    }
                    // * timer.stop();
                    // * std::cout << "fitPolinomial Time: " << timer.getTimeMilli() << " ms" << std::endl;
                    #if 1 == ENABLE_TCP_OUTPUT 
                        this->drawLines(outputImage,lines,cv::Scalar(0, 0, 255));
                        //this->laptopTCP.sendFrame(outputImage);
                    #endif
                    #if 1 != ENABLE_CAMERA_CALIBRATION 
                    for (int i = 0; i < lines.size(); i++){
                        if(this->isIntersection && lines[i][0].y >= frameHeight * this->lineStartPointY){
                            this->isPastIntersection = true;
                            this->isIntersection = false;
                        }
                        if(!this->isIntersection){
                            lines[i] = this->perspectiveChangeLine(lines[i], this->MatrixBirdsEyeView);
                            this->isIntersection = this->removeHorizontalIf90Turn(lines[i]);
                            this->extendLineToEdges(lines[i], birdsEyeViewWidth, birdsEyeViewHeight);
                            lines[i] = this->evenlySpacePoints(lines[i], curveSamplePoints);
                            if (this->isIntersection) {
                                std::vector<cv::Point2f> temp;
                                temp = lines[i]; 
                                lines.clear();
                                lines.emplace_back(temp); 
                                break;
                            }
                        }

                    }

                    /*
                        If car finds intersection it looks for the middle line 
                        and keeps middle till it goes past intersection.
                        Assigns middle line only once!
                    */
                
                    if (this->isPastIntersection){
                        // * timer.start();
                        this->getLeftRightLines(lines,this->leftLine,this->rightLine);
                        // * timer.stop();
                        // * std::cout << "getLeftRightLines Time: " << timer.getTimeMilli() << " ms" << std::endl;
                        
                        // * timer.start();
                        this->allMidPoints = this->findMiddle(this->leftLine,this->rightLine,birdsEyeViewWidth,birdsEyeViewHeight);
                        // * timer.stop();
                        // * std::cout << "findMiddle Time: " << timer.getTimeMilli() << " ms" << std::endl;
                        //std::cout << "Was here\n";
                        // Used to handle the case when no point is found in the middle of the intersection
                        if (this->isIntersection)
                                this->isPastIntersection = false;
                        
                    }
            
                    #endif
                    #if 1 == ENABLE_CAMERA_CALIBRATION 
                        // TO CALIBRATE CAMERA
                        cv::Point2f result;
                        std::vector<cv::Point2f> interpolatedPoints;
                        
                        #if 1 == ENABLE_TCP_OUTPUT 
                            this->laptopTCP.sendFrame(frame);
                        #endif

                        this->getLeftRightLines(lines,this->leftLine,this->rightLine);
                        this->extendLineToEdges(this->leftLine, this->frameWidth, this->frameHeight + distanceErrorFromChassis);
                        this->extendLineToEdges(this->rightLine, this->frameWidth, this->frameHeight + distanceErrorFromChassis);
                        this->allMidPoints = this->findMiddle(this->leftLine,this->rightLine,birdsEyeViewWidth,birdsEyeViewHeight);

                        interpolatedPoints.push_back(interpolateClosestPoints(this->leftLine, calibrateTopLine));
                        interpolatedPoints.push_back(interpolateClosestPoints(this->leftLine, calibrateBottomLine));
                        interpolatedPoints.push_back(interpolateClosestPoints(this->rightLine, calibrateTopLine));
                        interpolatedPoints.push_back(interpolateClosestPoints(this->rightLine, calibrateBottomLine));
                        interpolatedPoints.push_back(interpolateClosestPoints(this->allMidPoints, this->frameHeight + distanceErrorFromChassis));
                        // Write interpolated points to a TXT file
                        
                        // Show Data on frame
                        this->writePointsToTxt(interpolatedPoints, "interpolated_points.txt");
                        this->initPerspectiveVariables();
                        
                        #if 1 == ENABLE_TCP_OUTPUT 
                            this->drawPoints(outputImage, srcPoints, cv::Scalar(0, 255, 0));
                            this->drawHorizontalFromHeight(outputImage,calibrateTopLine,cv::Scalar(255, 255, 255));
                            this->drawHorizontalFromHeight(outputImage,calibrateBottomLine,cv::Scalar(255, 255, 255));
                            this->laptopTCP.sendFrame(outputImage);
                        #endif
                    #else 
                    
                        //radiusIncrease(lookAheadDistance);                
                        //pointMoveAcrossFrame(carInFramePositionBirdsEye, carTopPoint);
                    
                        // * timer.start();
                                            
                        lookAheadDistance = k * speed;
                        lookAheadDistance = std::max(minLookAhead, std::min(maxLookAhead, lookAheadDistance));
                        
                        double tempLookAheadDistance = shortestDistanceToCurve(allMidPoints, carInFramePositionBirdsEye, lookAheadDistance);
                        // * timer.stop();
                        // * std::cout << "shortestDistanceToCurve Time: " << timer.getTimeMilli() << " ms" << std::endl;
                        // Create a frame of the desired size
                        cv::Size frameSize(birdsEyeViewWidth, birdsEyeViewHeight);

                        // Initialize an empty image (black by default)
                        cv::Mat birdEyeViewWithPoints = cv::Mat::zeros(frameSize, CV_8UC3); // 3 channels (color)

                        // Find intersections
                        // * timer.start();
                        cv::Point2f lookAheadPoint = findHighestIntersection(allMidPoints, carInFramePositionBirdsEye, tempLookAheadDistance);
                        // * timer.stop();
                        // * std::cout << "findHighestIntersection Time: " << timer.getTimeMilli() << " ms" << std::endl;
                    
                        #if 1 == ENABLE_TCP_OUTPUT 
                            cv::line(birdEyeViewWithPoints, carInFramePositionBirdsEye, lookAheadPoint, cv::Scalar(123, 10, 255), 2); 
                            cv::line(birdEyeViewWithPoints, carInFramePositionBirdsEye, carTopPoint, cv::Scalar(123, 10, 255), 2);  
                            
                            cv::circle(birdEyeViewWithPoints, lookAheadPoint, 5, cv::Scalar(254, 34, 169), -1);
                            this->drawCircle(birdEyeViewWithPoints, carInFramePositionBirdsEye, tempLookAheadDistance, cv::Scalar(254, 34, 169));
                            cv::circle(birdEyeViewWithPoints, carInFramePositionBirdsEye, 5, cv::Scalar(254, 34, 169), -1);
                        
                            this->drawPoints(birdEyeViewWithPoints, dstPoints, cv::Scalar(0, 255, 255));
                            this->drawLineVector(birdEyeViewWithPoints,leftLine,cv::Scalar(0, 255, 0));
                            this->drawLineVector(birdEyeViewWithPoints,allMidPoints,cv::Scalar(255, 255, 255));
                            this->drawLineVector(birdEyeViewWithPoints,rightLine,cv::Scalar(0, 0, 255));
                            this->laptopTCP.sendFrame(birdEyeViewWithPoints);


                            std::vector<cv::Point2f> l_leftLine = this->perspectiveChangeLine(this->leftLine, MatrixInverseBirdsEyeView);
                            std::vector<cv::Point2f> l_rightLine = this->perspectiveChangeLine(this->rightLine, MatrixInverseBirdsEyeView);
                            std::vector<cv::Point2f> l_allMidPoints = this->perspectiveChangeLine(this->allMidPoints, MatrixInverseBirdsEyeView);
                            cv::Point2f l_lookAheadPoint = this->perspectiveChangePoint(lookAheadPoint, MatrixInverseBirdsEyeView);

                            outputImage = cv::Mat::zeros(frame.size(),CV_8UC3);

                            const int rowTopCutOffThreshold = static_cast<int>(outputImage.rows * topCutOffPercentageCustomConnected);
                            const int rowLineStartThreshold = static_cast<int>(outputImage.rows * lineBottomStartRangeCustomConnected);  
                            
                            this->drawHorizontalFromHeight(outputImage,rowTopCutOffThreshold,cv::Scalar(50, 20, 255));
                            this->drawHorizontalFromHeight(outputImage,rowLineStartThreshold,cv::Scalar(50, 20, 255));

                            this->drawPoints(outputImage, srcPoints, cv::Scalar(0, 255, 255));
                        
                            std::vector<cv::Point2f> horizontalLine;
                            horizontalLine.push_back(srcPoints[1]);
                            horizontalLine.push_back(srcPoints[3]);
                        
                            this->drawLineVector(outputImage,horizontalLine,cv::Scalar(0, 255, 255));

                            cv::circle(outputImage, l_lookAheadPoint, 5, cv::Scalar(254, 34, 169), -1);
                            cv::circle(outputImage, carInFramePosition, 5, cv::Scalar(254, 34, 169), -1);
                        
                            this->drawPoints(outputImage, l_leftLine, cv::Scalar(0, 255, 0));
                            this->drawPoints(outputImage, l_rightLine, cv::Scalar(0, 0, 255));
                            this->drawLineVector(outputImage,l_leftLine,cv::Scalar(0, 255, 0));
                            this->drawLineVector(outputImage,l_allMidPoints,cv::Scalar(255, 255, 255));
                            this->drawLineVector(outputImage,l_rightLine,cv::Scalar(0, 0, 255));
                        
                            //laptopTCP.sendFrame(outputImage);

                            cv::cvtColor(frame, frame, cv::COLOR_GRAY2BGR);

                            // Add the visualization (outputImage) on top of the original frame
                            cv::Mat overlayedImage;
                            cv::addWeighted(frame, overlayFrameWeight, outputImage, overlayFrameWeight, 0, overlayedImage);
                            laptopTCP.sendFrame(overlayedImage);
                        #endif
                        
                        // * timer.start();
                        double angle = calculateSignedAngle(carTopPoint, carInFramePositionBirdsEye, lookAheadPoint);
                        // * timer.stop();
                        // * std::cout << "calculateSignedAngle Time: " << timer.getTimeMilli() << " ms" << std::endl;
                        // * timer.start();
                        double steeringAngleServo = calculateServoValue(speed, angle, lookAheadDistance);
                        // * timer.stop();
                        // * std::cout << "calculateServoValue Time: " << timer.getTimeMilli() << " ms" << std::endl;
                        std::cout << "Calculated Steering Angle: " << angle << std::endl;
                        std::cout << "Calculated steeringAngleServo: " << steeringAngleServo << std::endl;
                                                
                        // * timer.start();
                        serial.writeToSerial(std::to_string(steeringAngleServo));
                        // * timer.stop();
                        // * std::cout << "writeToSerial Time: " << timer.getTimeMilli() << " ms" << std::endl;
                    #endif
                #endif
            }
        } catch (const std::exception& e) {
            std::cerr << "Error during frame processing: " << e.what() << std::endl;
        }
    }
}

// Retrieve the latest frame for processing
cv::Mat CameraProcessing::getLatestFrame() {
    std::lock_guard<std::mutex> lock(frameMutex);
    return latestFrame.clone();
}
        

// Resize the image to the specified dimensions
cv::Mat CameraProcessing::resizeImage(const cv::Mat& frame, int providedFrameWidth, int providedFrameHeight) {
    cv::Mat resizedFrame;
    cv::resize(frame, resizedFrame, cv::Size(providedFrameWidth, providedFrameHeight));

    this->frameWidth = providedFrameWidth;
    this->frameHeight = providedFrameHeight;

    return resizedFrame;
}
        
// Cuts pixel rows from image's top part
cv::Mat CameraProcessing::cropFrameTop(const cv::Mat& frame, double l_topCutOffPercentage) {
    if (0 == l_topCutOffPercentage) {
        return frame;
    }

    if (l_topCutOffPercentage < 0.0 || l_topCutOffPercentage >= 1.0) {
        throw std::invalid_argument("Cut Height must be between 0.0 and 1.0");
    }

    this->frameWidth = frame.cols;
    this->frameHeight = frame.rows;

    int cutOffHeight = frame.rows * l_topCutOffPercentage;

    int newFrameHeight = frame.rows - cutOffHeight;
    cv::Rect roi(0, cutOffHeight, this->frameWidth, newFrameHeight);

    this->frameHeight = newFrameHeight;
    return frame(roi);
}


cv::Mat CameraProcessing::skeletonizeFrame(cv::Mat& thresholdedImage) {
    cv::Mat skeleton(cv::Mat::zeros(thresholdedImage.size(), CV_8UC1));
    cv::Mat temp, eroded;
    cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));

    while (true) {
        cv::erode(thresholdedImage, eroded, element);
        cv::dilate(eroded, temp, element);
        cv::subtract(thresholdedImage, temp, temp);
        cv::bitwise_or(skeleton, temp, skeleton);
        eroded.copyTo(thresholdedImage);

        if (cv::countNonZero(thresholdedImage) == 0) {
            break;
        }
    }

    return skeleton;
}
// Apply color segmentation to isolate specific features in the image
cv::Mat CameraProcessing::segmentEdges(const cv::Mat& frame) {
    cv::Mat thresholdFrame;
    cv::threshold(frame, thresholdFrame, thresholdValue, maxThresholdValue, cv::THRESH_BINARY);
    cv::bitwise_not(thresholdFrame, thresholdFrame);
    return thresholdFrame;
}
        
// Function to find lines in a skeletonized frame
std::vector<std::vector<cv::Point2f>> CameraProcessing::findLines(const cv::Mat& thresholdedImage){

    // Perform connected component analysis
    cv::Mat labels;
    std::vector<std::vector<cv::Point2f>> lines;
    int numLabels = customConnectedComponentsWithThreshold(thresholdedImage, labels, 6,lines);
    
    // Return only the first 4 lines (or fewer if there are less than 4)
    if (lines.size() > 4) {
        lines.resize(4);
    }
    
    return lines;
}
        
int CameraProcessing::customConnectedComponentsWithThreshold(const cv::Mat& binaryImage, cv::Mat& labelImage, int radius, std::vector<std::vector<cv::Point2f>>& lines)
{
    CV_Assert(binaryImage.type() == CV_8UC1);
    labelImage = cv::Mat::zeros(binaryImage.size(), CV_32S); // Initialize label matrix
    lines.clear(); // Clear any existing lines

    const int rowTopCutOffThreshold = static_cast<int>(binaryImage.rows * topCutOffPercentageCustomConnected); 
    const int rowLineStartThreshold = static_cast<int>(binaryImage.rows * lineBottomStartRangeCustomConnected); 

    int label = 1; // Start labeling from 1
    std::vector<cv::Point2f> neighborhood = generateNeighborhood(radius); // Generate dynamic neighborhood
    int pixelCount;

    for (int y = binaryImage.rows - 1; y >= rowTopCutOffThreshold; --y) { // Bottom to top
        for (int x = 0; x < binaryImage.cols; ++x) {  // Left to right
            if (binaryImage.at<uchar>(y, x) == 255 && labelImage.at<int>(y, x) == 0) {
                if (y <= rowLineStartThreshold) {
                    continue;
                }

                std::queue<cv::Point2f> queue;
                queue.push(cv::Point2f(x, y));
                labelImage.at<int>(y, x) = label;

                pixelCount = 0;
                std::vector<cv::Point2f> componentPixels;

                while (!queue.empty()) {
                    cv::Point2f p = queue.front();
                    queue.pop();
                    pixelCount++;
                    componentPixels.push_back(p);

                    for (const auto& offset : neighborhood) {
                        int nx = p.x + offset.x;
                        int ny = p.y + offset.y;

                        if (nx >= 0 && ny >= rowTopCutOffThreshold  && nx < binaryImage.cols && ny < binaryImage.rows) {
                            if (binaryImage.at<uchar>(ny, nx) == 255 && labelImage.at<int>(ny, nx) == 0) {
                                labelImage.at<int>(ny, nx) = label;
                                queue.push(cv::Point2f(nx, ny));
                            }
                        }
                    }
                }

                if (pixelCount < lineMinPixelCount) {
                    for (const auto& p : componentPixels) {
                        labelImage.at<int>(p.y, p.x) = 0;
                    }
                } else {
                    lines.push_back(std::move(componentPixels));
                    label++;
                }
            }
        }
    }

    return label - 1;
}
// Function to smooth a vector of points using a moving average filter used in fitPolinomial
std::vector<cv::Point2f> CameraProcessing::smoothPoints(const std::vector<cv::Point2f>& points, int windowSize) {
    
    std::vector<cv::Point2f> smoothedPoints;

    if (windowSize < 1) {
        std::cerr << "Window size must be greater than or equal to 1." << std::endl;
        return points;
    }

    for (int i = 0; i < points.size(); ++i) {
        int sumX = 0, sumY = 0;
        int count = 0;


        // Window range must be within bounds of the points vector
        int start = std::max(0, (int)i - windowSize / 2);
        int end = std::min((int)points.size(), (int)i + windowSize / 2 + 1);

        // Calculate the average of neighboring points
        for (int j = start; j < end; ++j) {
            sumX += points[j].x;
            sumY += points[j].y;
            count++;
        }

        if (count > 0) {
            smoothedPoints.push_back(cv::Point2f(sumX / count, sumY / count));
        }
    }

    return smoothedPoints;
}
// Function to fit a polynomial to a set of points and return interpolated points
std::vector<cv::Point2f> CameraProcessing::fitPolinomial(const std::vector<cv::Point2f>& line, bool isMiddleLine) {

    std::vector<cv::Point2f> smoothedLine;

    if(2 == line.size()){
        return line;
    }
    
    if(!isMiddleLine){
        // Smooth the points of the current line using the moving average filter
        smoothedLine = smoothPoints(line, fitPolyWindowSize);
    }else{
        smoothedLine = line;
    }
    // Approximate the contours to further smooth them
    cv::approxPolyDP(smoothedLine, smoothedLine, fitPolyEpsilon, false);  // false = open curve  
    return smoothedLine;
}

/*
    Rises to the right (top-right):     m<0 (negative slope)
    Falls to the right (bottom-right):  m>0 (positive slope)
    Rises to the left (top-left):       m>0 (positive slope)
    Falls to the left (bottom-left):    m<0 (negative slope)
*/
void CameraProcessing::extendLineToEdges(std::vector<cv::Point2f>& line, int providedFrameWidth, int providedFrameHeight) {
    // Ensure the line has at least two points
    if (line.size() < 2) {
        std::cerr << "Line must have at least two points to extend." << std::endl;
        return;
    }
    cv::Point2f firstBottomPoint;
    cv::Point2f secondBottomPoint;
    cv::Point2f firstTopPoint;
    cv::Point2f secondTopPoint;
    

    // Get the second-to-last and last points to calculate the slope at the top end
    firstTopPoint = line.back();
    secondTopPoint = line[line.size() - 2];
    // Get the first and second points to calculate the slope at the bottom end
    firstBottomPoint = line[0];
    secondBottomPoint = line[1];
    

    // Calculate the slope for extending the line at the bottom end
    double deltaXBottom = secondBottomPoint.x - firstBottomPoint.x;
    double deltaYBottom = secondBottomPoint.y - firstBottomPoint.y;
    
    // Handle vertical lines (deltaX == 0)
    if (deltaXBottom == 0) {
        deltaXBottom = 1e-6f; // Avoid division by zero
    }
    double slopeBottom = deltaYBottom / deltaXBottom;

    // Extend the line to the bottom of the frame (y = providedFrameHeight)
    double xBottom = firstBottomPoint.x + ((providedFrameHeight - firstBottomPoint.y) / slopeBottom);
    if (xBottom < 0 - providedFrameWidth) xBottom = -providedFrameWidth; // Manually constrain to 2 times the left edge in case line is too long
    if (xBottom >= 2 * providedFrameWidth) xBottom = 2 * providedFrameWidth; // Manually constrain to 2 times the right edge in case line is too long
    cv::Point2f extendedBottom(static_cast<int>(xBottom), providedFrameHeight);

    // Add the extended point to the line
    line.insert(line.begin(), extendedBottom); // Add to the beginning
    

    double deltaXTop = firstTopPoint.x - secondTopPoint.x;
    double deltaYTop = firstTopPoint.y - secondTopPoint.y;
    
    // Handle vertical lines (deltaX == 0)
    if (deltaXTop == 0) {
        deltaXTop = 1e-6f; // Avoid division by zero
    }
    double slopeTop = deltaYTop / deltaXTop;
    
    if(slopeTop > 0.15 || slopeTop  < -0.15){
        
        // Extend the line to the top of the frame (y = 0)
        double xTop = firstTopPoint.x - (firstTopPoint.y / slopeTop);
        if (xTop < 0 - providedFrameWidth) xTop = -providedFrameWidth; // Manually constrain to 2 times the left edge
        if (xTop >= 2 * providedFrameWidth) xTop = 2 * providedFrameWidth; // Manually constrain to 2 times the right edge
        cv::Point2f extendedTop(static_cast<int>(xTop), 0);
        // Add the extended points to the line
        line.push_back(extendedTop);                    // Add to the end
    }
}
// Function to evenly space points along the curve based on arc length
std::vector<cv::Point2f> CameraProcessing::evenlySpacePoints(const std::vector<cv::Point2f>& line, int numPoints) {
    std::vector<cv::Point2f> spacedPoints;
    
    // Step 1: Calculate the total arc length of the contour
    std::vector<double> arcLengths(line.size(), 0.0f);
    for (int i = 1; i < line.size(); ++i) {
        arcLengths[i] = arcLengths[i - 1] + static_cast<double>(euclideanDistance(line[i - 1], line[i]));
    }
    double totalArcLength = arcLengths.back();

    // Step 2: Define the target spacing between points
    double spacing = totalArcLength / (numPoints - 1);

    // Step 3: Interpolate points based on arc length
    spacedPoints.push_back(line.front());  // Add the first point
    double currentArcLength = spacing;

    for (int i = 1; i < numPoints - 1; ++i) {
        // Find where the currentArcLength falls in the original contour
        for (int j = 1; j < line.size(); ++j) {
            if (arcLengths[j] >= currentArcLength) {
                // Linear interpolation between points j-1 and j
                double ratio = (currentArcLength - arcLengths[j - 1]) / (arcLengths[j] - arcLengths[j - 1]);
                double x = line[j - 1].x + ratio * (line[j].x - line[j - 1].x);
                double y = line[j - 1].y + ratio * (line[j].y - line[j - 1].y);
                spacedPoints.push_back(cv::Point2f(x, y));
                break;
            }
        }
        currentArcLength += spacing;
    }

    spacedPoints.push_back(line.back());  // Add the last point

    return spacedPoints;
}
bool CameraProcessing::are2PointsHorizontal(const cv::Point2f& p1, const cv::Point2f& p2, double slopeThreshold) 
{
    
    double deltaX = p2.x - p1.x;
    double deltaY = p2.y - p1.y;

    // Avoid division by zero (in case of vertical lines)
    if (deltaX == 0) {
        return false;  // It's a vertical line
    }

    // Calculate the slope
    double slope = deltaY / deltaX;
    // Check if the slope is close to zero (indicating a horizontal line)
    return std::abs(slope) <= slopeThreshold;
}
// Function to remove horizontal sections if a 90-degree turn is detected
bool CameraProcessing::removeHorizontalIf90Turn(std::vector<cv::Point2f>& line)
{
    bool has90DegreeTurn = false;
    std::vector<cv::Point2f> result;

    double maxAngle = 0.0; // Variable to store the maximum angle

    // Check for 90-degree turns along the line
    for (int i = 1; i < line.size() - 1; ++i) 
    {
        double angle = std::abs(calculateSignedAngle(line[i - 1], line[i], line[i + 1])* 180.0 / CV_PI);
        maxAngle = std::max(maxAngle, angle); // Update maxAngle if the current angle is larger

        if (angle > min90DegreeAngleRange && angle < max90DegreeAngleRange) // Close to 90 degrees
        {  
            has90DegreeTurn = true;
            //std::cout << "Angle(has90DegreeTurn): " << angle << std::endl;
            break; // You may break early if you're only checking for 90-degree turns
        }
    }

    // Print the maximum angle after the loop
    std::cout << "Maximum angle detected: " << maxAngle << " degrees" << std::endl;


    // If there is a 90-degree turn, remove the horizontal parts
    if (has90DegreeTurn) 
    {
        
        for (int i = 1; i < line.size(); ++i) 
        {
            if (!are2PointsHorizontal(line[i - 1], line[i], horizontalSlopeThreshold)) 
            {
                // Add the starting point of the non-horizontal segment
                if (result.empty() || result.back() != line[i - 1]) 
                {
                    result.push_back(line[i - 1]);
                }

                // Add the endpoint of the non-horizontal segment
                if (result.empty() || result.back() != line[i]) 
                {
                    result.push_back(line[i]);
                }
            }
        }

        //std::cout << "Line has 90DegreeTurn" << std::endl;

        // Assign new line
        line = result;
    }
    else
    {
        //std::cout << "Line is Normal" << std::endl;
    }
    
    return has90DegreeTurn;

}
// Function that handles 2 Lines, 1 Line and No line cases
void CameraProcessing::getLeftRightLines(const std::vector<std::vector<cv::Point2f>>& lines, std::vector<cv::Point2f>& leftFitted, std::vector<cv::Point2f>& rightFitted)
{

    cv::Point2f firstPointLineA = undefinedPoint;
    cv::Point2f firstPointLineB = undefinedPoint;
    cv::Point2f shiftedPoint;
    double deltaX;
    double deltaY;
    double slope;
    double alpha = 1.0;

    /*
        If we have 2 line it looks to see which one's X value is smaller
        because the frame start from 0,0 and it get's extended to the right
        then the smaller value of X is the left line

        firstPointLineA at first is choosed randomly but after it is set to left line first point
    */
   
    // std::cout << " lines.size(): " << lines.size() << "\n";
    // std::cout << " firstPointLeftLine: " << firstPointLeftLine << "\n";
    // std::cout << " firstPointRightLine: " << firstPointRightLine << "\n";
    // std::cout << " firstPointSingleLine: " << firstPointSingleLine << "\n";
    // std::cout << " ---------------------------------\n";


    if (lines.size() >= 2) {
        firstPointLineA = lines[0][0];
        firstPointLineB = lines[1][0];

        // Weighted distances
        double weightedDistAtoLeft = euclideanDistance(firstPointLineA, firstPointLeftLine) 
                                    + alpha * abs(firstPointLineA.y - firstPointLeftLine.y);
        double weightedDistAtoRight = euclideanDistance(firstPointLineA, firstPointRightLine) 
                                    + alpha * abs(firstPointLineA.y - firstPointRightLine.y);
        double weightedDistBtoLeft = euclideanDistance(firstPointLineB, firstPointLeftLine) 
                                    + alpha * abs(firstPointLineB.y - firstPointLeftLine.y);
        double weightedDistBtoRight = euclideanDistance(firstPointLineB, firstPointRightLine) 
                                    + alpha * abs(firstPointLineB.y - firstPointRightLine.y);


        std::cout <<" weightedDistAtoLeft" << weightedDistAtoLeft << std::endl;
        std::cout <<" weightedDistAtoRight" << weightedDistAtoRight << std::endl;
        std::cout <<" weightedDistAtoLeft" << weightedDistBtoLeft << std::endl;
        std::cout <<" weightedDistAtoRight" << weightedDistBtoRight << std::endl;
        // Check for impostors
        if ((weightedDistAtoLeft < weightedDistAtoRight && weightedDistBtoLeft < weightedDistBtoRight) || 
            (weightedDistAtoRight < weightedDistAtoLeft && weightedDistBtoRight < weightedDistBtoLeft)) 
        {
            std::cout << "Distances: " << std::endl;
            std::cout << "weightedDistAtoLeft: " << weightedDistAtoLeft << std::endl;
            std::cout << "weightedDistAtoRight: " << weightedDistAtoRight << std::endl;
            std::cout << "weightedDistBtoLeft: " << weightedDistBtoLeft << std::endl;
            std::cout << "weightedDistBtoRight: " << weightedDistBtoRight << std::endl;

            double smallestDistance = weightedDistAtoLeft;
            std::string smallestType = "AtoLeft";

            // Step 3: Compare each distance to find the smallest
            if (weightedDistAtoRight < smallestDistance) {
                smallestDistance = weightedDistAtoRight;
                smallestType = "AtoRight";
            }
            if (weightedDistBtoLeft < smallestDistance) {
                smallestDistance = weightedDistBtoLeft;
                smallestType = "BtoLeft";
            }
            if (weightedDistBtoRight < smallestDistance) {
                smallestDistance = weightedDistBtoRight;
                smallestType = "BtoRight";
            }

            std::cout << "Smallest distance: " << smallestDistance << " (" << smallestType << ")" << std::endl;

            if (smallestType == "AtoLeft") {
                // Line A is closest to the left
                std::cout << "Classifying Line A as the left line." << std::endl;
                leftFitted = lines[0];
                rightFitted.clear();
                for (const auto& point : leftFitted) {
                    shiftedPoint = cv::Point2f(point.x + trackLaneWidthInPixel, point.y); // Mirror left to right
                    rightFitted.push_back(shiftedPoint);
                }
                firstPointLeftLine = firstPointLineA; // Update historical left point
                firstPointRightLine = rightFitted[0]; // Update historical right point
            } else if (smallestType == "AtoRight") {
                // Line A is closest to the right
                rightFitted = lines[0];
                leftFitted.clear();
                for (const auto& point : rightFitted) {
                    shiftedPoint = cv::Point2f(point.x - trackLaneWidthInPixel, point.y); // Mirror right to left
                    leftFitted.push_back(shiftedPoint);
                }
                firstPointRightLine = firstPointLineA; // Update historical right point
                firstPointLeftLine = leftFitted[0]; // Update historical left point
            } else if (smallestType == "BtoLeft") {
                // Line B is closest to the left
                leftFitted = lines[1];
                rightFitted.clear();
                for (const auto& point : leftFitted) {
                    shiftedPoint = cv::Point2f(point.x + trackLaneWidthInPixel, point.y); // Mirror left to right
                    rightFitted.push_back(shiftedPoint);
                }
                firstPointLeftLine = firstPointLineB; // Update historical left point
                firstPointRightLine = rightFitted[0]; // Update historical right point
            } else if (smallestType == "BtoRight") {
                // Line B is closest to the right
                rightFitted = lines[1];
                leftFitted.clear();
                for (const auto& point : rightFitted) {
                    shiftedPoint = cv::Point2f(point.x - trackLaneWidthInPixel, point.y); // Mirror right to left
                    leftFitted.push_back(shiftedPoint);
                }
                firstPointRightLine = firstPointLineB; // Update historical right point
                firstPointLeftLine = leftFitted[0]; // Update historical left point
            }
        } else {
            // Correct classification
            if (weightedDistAtoLeft < weightedDistAtoRight) {
                leftFitted = lines[0];
                rightFitted = lines[1];
                firstPointLeftLine = firstPointLineA;
                firstPointRightLine = firstPointLineB;
            } else {
                leftFitted = lines[1];
                rightFitted = lines[0];
                firstPointLeftLine = firstPointLineB;
                firstPointRightLine = firstPointLineA;
            }
        }
    }



    /*
        If we have one line and history then decide to which starting point is closer:
            - If it is to the left then duplicate a line to it's left and lower
            - If it is to the right then duplicat a line to it's right and lower

        If we have one line and no history then decide based on slope (corner case)
    */
    /*else if (1 == lines.size())
    {
        // Compare to based on slope of first and last points
        // If it has an orientation to the left it is left otherwise rightv
        cv::Point2f pointBack(lines[0].back().x,lines[0].back().y);
        cv::Point2f pointFront(lines[0][0].x,lines[0][0].y);

        pointBack = perspectiveChangePoint(pointBack, MatrixInverseBirdsEyeView);
        pointFront = perspectiveChangePoint(pointFront, MatrixInverseBirdsEyeView);

        double deltaX = pointBack.x - pointFront.x;
        double deltaY = pointBack.y - pointFront.y;
        // Handle vertical lines (deltaX == 0)
        if (deltaX == 0) {
            deltaX = 1e-6f; // Avoid division by zero
        }
        double slope = deltaY / deltaX;

        // If slope rises to the right (top-right) then we have left line
        if(slope < 0)
        {
            leftFitted = lines[0];
            rightFitted.clear();
            for (const auto& point : leftFitted) {
                shiftedPoint = cv::Point2f(point.x + trackLaneWidthInPixel, point.y);
                rightFitted.push_back(shiftedPoint);
            }
            firstPointLeftLine = leftFitted[0];
            firstPointRightLine = rightFitted[0];
        } 
        else
        {
            rightFitted = lines[0];
            leftFitted.clear();
            for (const auto& point : rightFitted) {
                shiftedPoint = cv::Point2f(point.x - trackLaneWidthInPixel, point.y);
                leftFitted.push_back(shiftedPoint);
            }
            
            firstPointLeftLine = leftFitted[0];
            firstPointRightLine = rightFitted[0];
        }            
    }*/

    // Almost good approach
    /*
        else if (1 == lines.size())
        {
            // Get the first, second, and last points of the line
            cv::Point2f pointFront(lines[0][0].x, lines[0][0].y);       // First point of the line
            cv::Point2f pointSecond(lines[0][1].x, lines[0][1].y);      // Second point of the line
            cv::Point2f pointBack(lines[0].back().x, lines[0].back().y); // Last point of the line

            // Calculate slope using the first two points
            double deltaX = pointSecond.x - pointFront.x;
            double deltaY = pointSecond.y - pointFront.y;
            if (deltaX == 0) {
                deltaX = 1e-6f; // Avoid division by zero
            }
            double slope = deltaY / deltaX;

            // Output debug information
            std::cout << "Slope (First 2 Points): " << slope << std::endl;

            if (std::abs(slope) < 1e-3) { // Ambiguous case: Slope is approximately 0
                std::cout << "Ambiguous slope detected (near 0). Checking last point..." << std::endl;

                // History check: Compare the last point to historical positions
                double distToLeftHistory = euclideanDistance(pointBack, firstPointLeftLine);
                double distToRightHistory = euclideanDistance(pointBack, firstPointRightLine);

                // Output debug information
                std::cout << "Distance to Left History: " << distToLeftHistory << std::endl;
                std::cout << "Distance to Right History: " << distToRightHistory << std::endl;

                // Classify the line based on the last point
                if (distToLeftHistory < distToRightHistory) {
                    // Single line is closer to historical left, so it is the right line
                    rightFitted = lines[0];
                    leftFitted.clear();
                    for (const auto& point : rightFitted) {
                        shiftedPoint = cv::Point2f(point.x - trackLaneWidthInPixel, point.y);
                        leftFitted.push_back(shiftedPoint);
                    }
                    firstPointRightLine = pointFront; // Update historical right point
                    firstPointLeftLine = leftFitted[0]; // Update historical left point
                } else {
                    // Single line is closer to historical right, so it is the left line
                    leftFitted = lines[0];
                    rightFitted.clear();
                    for (const auto& point : leftFitted) {
                        shiftedPoint = cv::Point2f(point.x + trackLaneWidthInPixel, point.y);
                        rightFitted.push_back(shiftedPoint);
                    }
                    firstPointLeftLine = pointFront; // Update historical left point
                    firstPointRightLine = rightFitted[0]; // Update historical right point
                }
            } else {
                // Classify based on slope
                if (slope < 0) {
                    // Line rises to the left, so it is the left line
                    leftFitted = lines[0];
                    rightFitted.clear();
                    for (const auto& point : leftFitted) {
                        shiftedPoint = cv::Point2f(point.x + trackLaneWidthInPixel, point.y);
                        rightFitted.push_back(shiftedPoint);
                    }
                    firstPointLeftLine = pointFront; // Update historical left point
                    firstPointRightLine = rightFitted[0]; // Update historical right point
                } else {
                    // Line rises to the right, so it is the right line
                    rightFitted = lines[0];
                    leftFitted.clear();
                    for (const auto& point : rightFitted) {
                        shiftedPoint = cv::Point2f(point.x - trackLaneWidthInPixel, point.y);
                        leftFitted.push_back(shiftedPoint);
                    }
                    firstPointRightLine = pointFront; // Update historical right point
                    firstPointLeftLine = leftFitted[0]; // Update historical left point
                }
            }
        }
        */
        else if (1 == lines.size())
        {
            // Get the first and last points of the line
            cv::Point2f pointFront(lines[0][0].x, lines[0][0].y);       // First point of the line

            // Weighted distance comparison using the first point
            double weightedDistToLeft = euclideanDistance(pointFront, firstPointLeftLine) 
                                    + alpha * std::abs(pointFront.y - firstPointLeftLine.y);
            double weightedDistToRight = euclideanDistance(pointFront, firstPointRightLine) 
                                        + alpha * std::abs(pointFront.y - firstPointRightLine.y);

            // Output debug information
            std::cout << "Weighted Distance to Left: " << weightedDistToLeft << std::endl;
            std::cout << "Weighted Distance to Right: " << weightedDistToRight << std::endl;

            // Classify the line as left or right based on weighted distances
            if (weightedDistToLeft < weightedDistToRight) {
                // Single line is closer to historical left, so it is the left line
                leftFitted = lines[0];
                rightFitted.clear();
                for (const auto& point : leftFitted) {
                    shiftedPoint = cv::Point2f(point.x + trackLaneWidthInPixel, point.y);
                    rightFitted.push_back(shiftedPoint);
                }
                firstPointLeftLine = pointFront; // Update historical left point
                firstPointRightLine = rightFitted[0]; // Update historical right point
            } else {
                // Single line is closer to historical right, so it is the right line
                rightFitted = lines[0];
                leftFitted.clear();
                for (const auto& point : rightFitted) {
                    shiftedPoint = cv::Point2f(point.x - trackLaneWidthInPixel, point.y);
                    leftFitted.push_back(shiftedPoint);
                }
                firstPointRightLine = pointFront; // Update historical right point
                firstPointLeftLine = leftFitted[0]; // Update historical left point
            }
        }

        else {
        //firstPointLeftLine = undefinedPoint;
        //firstPointRightLine = undefinedPoint;

        if(!isIntersection){
            // TBD NO LINES, STOP CAR
        }
    }
}
std::vector<cv::Point2f> CameraProcessing::findMiddle(std::vector<cv::Point2f>& leftLine, std::vector<cv::Point2f>& rightLine, int providedFrameWidth, int providedFrameHeight) {
    std::vector<cv::Point2f> midPoints;
    for (size_t i = 0; i < std::min(leftLine.size(), rightLine.size()); ++i) {
        cv::Point2f midpoint((leftLine[i].x + rightLine[i].x) / 2, (leftLine[i].y + rightLine[i].y) / 2);
        midPoints.push_back(midpoint);
    }
    this->extendLineToEdges(midPoints, providedFrameWidth, providedFrameHeight);
    return midPoints;
}
        
cv::Point2f CameraProcessing::interpolateClosestPoints(const std::vector<cv::Point2f>& points, int targetY) {
    if (points.size() < 2) {
        throw std::invalid_argument("Not enough points to interpolate");
    }
    targetY -= 1;                                           // -1 used to set midpoint a bit above lower limit
    // Initialize variables to track the closest points
    cv::Point2f lower, upper;
    bool lowerFound = false, upperFound = false;

    for (const auto& point : points) {
        if (point.y <= targetY) {
            if (!lowerFound || point.y > lower.y) {
                lower = point;
                lowerFound = true;
            }
        }
        if (point.y >= targetY) {
            if (!upperFound || point.y < upper.y) {
                upper = point;
                upperFound = true;
            }
        }
    }

    // Ensure we found bounding points
    if (!lowerFound || !upperFound) {
        throw std::invalid_argument("Target Y is out of bounds of the given points");
    }

    // Linear interpolation to find x
    int interpolatedX = lower.x + (targetY - lower.y) * (upper.x - lower.x) / (upper.y - lower.y);
    return cv::Point2f(interpolatedX, targetY);
}
// Perspective transform related methods
void CameraProcessing::initPerspectiveVariables() {
    auto loadedPoints = readPointsFromTxt("interpolated_points.txt");

    this->srcPoints = {
        cv::Point2f(loadedPoints[0].x, loadedPoints[0].y),
        cv::Point2f(loadedPoints[1].x, loadedPoints[1].y),
        cv::Point2f(loadedPoints[2].x, loadedPoints[2].y),
        cv::Point2f(loadedPoints[3].x, loadedPoints[3].y)
    };

    dstPoints = {
        cv::Point2f(
            birdsEyeViewWidth / 2 - widthDstPoints / 2,
            birdsEyeViewHeight - heightDstPoints - distanceErrorFromChassis
        ), // Top-left corner
        cv::Point2f(
            birdsEyeViewWidth / 2 - widthDstPoints / 2,
            birdsEyeViewHeight - distanceErrorFromChassis
        ), // Bottom-left corner
        cv::Point2f(
            birdsEyeViewWidth / 2 + widthDstPoints / 2,
            birdsEyeViewHeight - heightDstPoints - distanceErrorFromChassis
        ), // Top-right corner
        cv::Point2f(
            birdsEyeViewWidth / 2 + widthDstPoints / 2,
            birdsEyeViewHeight - distanceErrorFromChassis
        ) // Bottom-right corner
    };

    this->trackLaneWidthInPixel = euclideanDistance(dstPoints[3], dstPoints[1]) + 20;   //Error mitigation +10
    this->MatrixBirdsEyeView = cv::getPerspectiveTransform(srcPoints, dstPoints);
    this->MatrixInverseBirdsEyeView = this->MatrixBirdsEyeView.inv();
    
    //std::cout << " carInFramePosition: " << carInFramePosition << "\n";
    //std::cout << " carInFramePositionBirdsEye: " << carInFramePositionBirdsEye << "\n";
    this->carInFramePosition = cv::Point2f(loadedPoints[4].x, loadedPoints[4].y);
    this->carInFramePositionBirdsEye = this->perspectiveChangePoint(this->carInFramePosition, this->MatrixBirdsEyeView);
    this->carTopPoint = cv::Point2f(this->carInFramePositionBirdsEye.x, 0.0);
    
    //std::cout << " carInFramePosition: " << carInFramePosition << "\n";
    //std::cout << " carInFramePositionBirdsEye: " << carInFramePositionBirdsEye << "\n";
}

cv::Point2f CameraProcessing::perspectiveChangePoint(const cv::Point2f& point, const cv::Mat& transformMatrix)
{
    std::vector<cv::Point2f> src = {point};
    std::vector<cv::Point2f> dst;
    cv::perspectiveTransform(src, dst, transformMatrix);
    return dst[0];
}

std::vector<cv::Point2f> CameraProcessing::perspectiveChangeLine(const std::vector<cv::Point2f>& line, const cv::Mat& transformMatrix)
{
    std::vector<cv::Point2f> transformedLine;
    cv::perspectiveTransform(line, transformedLine, transformMatrix);
    return transformedLine;
}

void CameraProcessing::saveImage(const std::string& filename, const cv::Mat& frame) {
    if (!cv::imwrite(filename, frame)) {
        std::cerr << "Failed to save the image!" << std::endl;
    }
}

void CameraProcessing::writePointsToTxt(const std::vector<cv::Point2f>& points, const std::string& filename) {
    std::ofstream file(filename);
    if (file.is_open()) {
        for (const auto& point : points) {
            file << point.x << " " << point.y << "\n";
        }
        file.close();
    } else {
        std::cerr << "Unable to open file for writing: " << filename << "\n";
    }
}

std::vector<cv::Point2f> CameraProcessing::readPointsFromTxt(const std::string& filename) {
    std::vector<cv::Point2f> points;
    std::ifstream file(filename);
    if (file.is_open()) {
        float x, y;
        while (file >> x >> y) {
            points.emplace_back(x, y);
        }
        file.close();
    } else {
        std::cerr << "Unable to open file for reading: " << filename << "\n";
    }
    return points;
}

void CameraProcessing::drawPoints(cv::Mat& frame, const std::vector<cv::Point2f>& points, const cv::Scalar& color)
{
    for (const auto& point : points) {
        cv::circle(frame, point, 5, color, -1);
    }
}

void CameraProcessing::drawHorizontalFromHeight(cv::Mat& frame, const int heightPoint, const cv::Scalar& color)
{
    cv::Point2f leftPoint(0, heightPoint);
    cv::Point2f RightPoint(frame.cols, heightPoint);
    cv::line(frame, leftPoint, RightPoint, color, 2);  // Green
}

void CameraProcessing::drawLineVector(cv::Mat& frame, const std::vector<cv::Point2f>& line, const cv::Scalar& color)
{
    for (int i = 1; i < line.size(); i++) {
        cv::line(frame, line[i-1], line[i], color, 2);  // Green
    }
}

void CameraProcessing::drawLines(cv::Mat& frame, const std::vector<std::vector<cv::Point2f>> lines, const cv::Scalar& color)
{
    for (int i = 0; i < lines.size(); ++i) {
        drawLineVector(frame,lines[i],color);
    }
}

void CameraProcessing::drawCircle(cv::Mat& image, const cv::Point2f& center, int radius, const cv::Scalar& color, int thickness)
{
    cv::circle(image, center, radius, color, thickness);
}

#endif
