#ifndef CAMERA_PROCESSING_HPP
#define CAMERA_PROCESSING_HPP

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/ximgproc.hpp>
#include <atomic>
#include <thread>
#include <mutex>
#include <vector>
#include <cmath>
#include <fstream>
#include <queue>
#include <chrono>
#include <unistd.h>
#include "config.h"
#include "MathFunctions/MathFunctions.hpp"
#include "TcpFunctions/TcpConnection.hpp"
#include "PurePursuitFunctions/PurePursuit.hpp"
#include <cstdlib> // Required for exit()

struct timerState{
    std::chrono::steady_clock::time_point startTime;
    bool startTimeEnabled = false;
};

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
        double trackLaneWidthInCm;
        double pixelSizeInCm;
        
        #if 1 == ENABLE_TCP_FRAMES 
            TcpConnection liveVideoFeedTCP{9999};
        #endif

        bool isValidLines = true;
        bool isIntersection = false;      // Used to check if we are really in an intersection
        bool is90DegreeLine = false;        // Used to check if we have an intersection on the racetrack, a crosspath

        bool isFinishLineDetected = false;
        /* NOTE: this boolean is used to block the frameProcessing algorithm after the box is closer then certain distance */
        bool isObjectCloserThanDistanceBeforeIssuesAppear = false;

        // Used in getLeftRightLines()
        cv::Point2f firstPointLeftLine = undefinedPoint;
        cv::Point2f firstPointRightLine = undefinedPoint;
        cv::Point2f firstPointSingleLine = undefinedPoint;

        int lastInterpolatedPointsSetup = 0;

        SharedConfig* config;
        PurePursuit ppObject;

    public:
        std::atomic<bool> running{false};
        // Default constructor
        CameraProcessing();
        // Constructor with parameters for image read
        CameraProcessing(std::shared_ptr<SharedConfig> global_config);
        // Constructor with camera parameters
        CameraProcessing(int cameraIndex, int width, int height, int fps, std::shared_ptr<SharedConfig> global_config);
        // Destructor stops capturing
        ~CameraProcessing();
        // Set camera parameters
        void setParameters(int cameraIndex, int width, int height, int fps, std::shared_ptr<SharedConfig> global_config);
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
        // Cuts pixel rows from image's bottom part
        cv::Mat cropFrameBottom(const cv::Mat& frame, double l_bottomCutOffPercentage);
        cv::Mat skeletonizeFrame(cv::Mat& thresholdedImage);
        // Apply color segmentation to isolate specific features in the image
        cv::Mat applyThreshold(const cv::Mat& frame);
        // Function to find lines in a skeletonized frame
        std::vector<std::vector<cv::Point2f>> findLines(const cv::Mat& thresholdedImage, int currentState, const int rowTopCutOffThreshold);        
        int customConnectedComponents(const cv::Mat& binaryImage, cv::Mat& labelImage, int radius,
             std::vector<std::vector<cv::Point2f>>& lines, int currentState, const int rowTopCutOffThreshold);
        // Function to smooth a vector of points using a moving average filter used in fitPolinomial
        std::vector<cv::Point2f> smoothPoints(const std::vector<cv::Point2f>& points, int windowSize);
        // Function to fit a polynomial to a set of points and return interpolated points
        std::vector<cv::Point2f> fitPolinomial(const std::vector<cv::Point2f>& line, bool isMiddleLine);
        void extendLineToEdges(std::vector<cv::Point2f>& line, int providedFrameWidth, int providedFrameHeight);
        // Function to evenly space points along the curve based on arc length
        std::vector<cv::Point2f> evenlySpacePoints(const std::vector<cv::Point2f>& line, int num_points);
        bool are2PointsHorizontal(const cv::Point2f& p1, const cv::Point2f& p2, double slopeThreshold = 1);
        // Function to remove horizontal sections if a 90-degree turn is detected
        bool removeHorizontalIf90Turn(std::vector<cv::Point2f>& line, int currentState);
        // Used in single line detection
        cv::Point2f normalize(const cv::Point2f& v);
        cv::Point2f rightNormal(const cv::Point2f& v);
        cv::Point2f leftNormal(const cv::Point2f& v);
        cv::Point2f ensureCorrectSide(const cv::Point2f& base, const cv::Point2f& offsetPoint, const cv::Point2f& historicalReference);
        // Function that handles 2 Lines, 1 Line and No line cases
        void getLeftRightLines(const std::vector<std::vector<cv::Point2f>>& lines, std::vector<cv::Point2f>& leftFitted, std::vector<cv::Point2f>& rightFitted); 
        std::vector<cv::Point2f> findMiddle(std::vector<cv::Point2f>& leftLine, std::vector<cv::Point2f>& rightLine, int providedFrameWidth, int providedFrameHeight);
        cv::Point2f interpolateClosestPoints(const std::vector<cv::Point2f>& points, int targetY);
        double calculateLineLength(const std::vector<cv::Point2f>& curve);
        uint16_t getMedian(uint16_t *array, int size);
        // Functions to read Specific Data From Recieved Data via Serial
        uint16_t getDistanceFromCarsBumper();

        // Perspective transform related methods
        void initPerspectiveVariables(std::string inputTxt);
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

        std::string createSerialString(bool isBlueStatusLedOn, bool isYellowStatusLedOn, bool isRadarEnabled);
        void CameraProcessing::handleRaceStart(ConfigStruct* config, timerState& startTimeState, PurePursuit& ppObject,
            int DEFAULT_EDF_FAN_CURRENT_SPEED, bool& isFinishLineDetected)
};

#endif