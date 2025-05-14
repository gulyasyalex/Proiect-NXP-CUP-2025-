#include "CameraFunctions/CameraProcessing.hpp"

// Default constructor
CameraProcessing::CameraProcessing() {}

// Constructor with camera parameters
CameraProcessing::CameraProcessing(int cameraIndex, int width, int height, int fps, std::shared_ptr<SharedConfig> global_config)
{
    setParameters(cameraIndex, width, height, fps, global_config);
}
CameraProcessing::CameraProcessing(std::shared_ptr<SharedConfig> global_config)
{
    this->config = global_config.get();
    ppObject.setParameters(global_config);
}

// Destructor stops capturing
CameraProcessing::~CameraProcessing() {
    stopCapture();
}

// Set camera parameters
void CameraProcessing::setParameters(int cameraIndex, int width, int height, int fps, std::shared_ptr<SharedConfig> global_config) {
    cap.open(cameraIndex, cv::CAP_V4L2);
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    cap.set(cv::CAP_PROP_FPS, fps);

    // Apply v4l2-ctl camera settings for manual exposure NXP BUCHAREST FIX
    /*std::string command = "v4l2-ctl -d /dev/video" + std::to_string(cameraIndex) +
                          " -c auto_exposure=1 -c exposure_time_absolute=16";*/
                          
    // Apply v4l2-ctl camera settings for manual exposure TIMISOARA
    std::string command = "v4l2-ctl -d /dev/video" + std::to_string(cameraIndex) +
                          " -c auto_exposure=1 -c exposure_time_absolute=15";
    system(command.c_str());

    double actualWidth = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    double actualHeight = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    double actualFPS = cap.get(cv::CAP_PROP_FPS);

    std::cout << "Camera settings:" << std::endl;
    std::cout << "  Frame width: " << actualWidth << std::endl;
    std::cout << "  Frame height: " << actualHeight << std::endl;
    std::cout << "  FPS: " << actualFPS << std::endl;

    this->config = global_config.get();
    ppObject.setParameters(global_config);

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
                std::string imagePath = "imagine03032025_01.jpg";
                //std::string imagePath = "imagine12022024_02.jpg";
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
            //saveImage("imagine03032025_01.jpg",frame);
            cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);

            std::deque<double> brightness_history;
            int window_size = 50;

            double frame_brightness = cv::mean(frame)[0];

            if (brightness_history.size() >= window_size) {
                brightness_history.pop_front();
            }
            brightness_history.push_back(frame_brightness);

            // Compute median
            std::vector<double> temp(brightness_history.begin(), brightness_history.end());

            std::sort(temp.begin(), temp.end());
            double median_brightness = temp[temp.size() / 2];
            //std::cout << "frame_brightness:" << frame_brightness << "\n";
            //std::cout << "median_brightness:" << median_brightness << "\n";
            if (std::abs(frame_brightness - median_brightness) > 55) {
                // Flicker detected: skip or discard this frame
                continue;
            }

            // TO CROP
            frame = cropFrameTop(frame, config->topImageCutPercentage);

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
    
    double exitingIntersectionCounter = 0;  

    bool isBlueStatusLedOn = false;
    bool isYellowStatusLedOn = false;
    bool isStartLineSetup = true;
    int rowTopCutOffThreshold;

    std::chrono::time_point<std::chrono::steady_clock> startTime;
    std::chrono::time_point<std::chrono::steady_clock> finishLineTime;

    bool startTimeEnabled = false;
    bool finishLineTimeEnabled = false;
    bool isSlowDownSpeedActivated = false;
    
    std::string serialString = "";
    
    bool isRadarEnabled = false;
    int stoppingDistanceBoxLidar = config->stoppingDistanceBoxFrontEnd + config->distanceSensorError; 
    std::vector<cv::Point2f> simplifiedLine;
    cv::Point2f midReferencePoint;

    double finishLineLeftAngle = 0;
    double finishLineRightAngle = 0;

    while (this->running) {
        try {
            this->lastInterpolatedPointsSetup = config->interpolatedPointsSetup;

            if (NEAR_VIEW_SETUP == config->interpolatedPointsSetup){
                this->initPerspectiveVariables("interpolated_points_NEAR.txt");
            }
            else if(FAR_VIEW_SETUP == config->interpolatedPointsSetup){
                this->initPerspectiveVariables("interpolated_points_FAR.txt");
            }else{
                this->initPerspectiveVariables("interpolated_points.txt");
            }
            cv::Mat frame;

            {
                std::lock_guard<std::mutex> lock(frameMutex);
                frame = this->latestFrame.clone(); // Always copy the latest frame
            }

            if (!frame.empty()) {
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

                /* NOTE: After finish line is detected we check if object is closer then "distanceBeforeIssuesAppear"
                 *       After that we just lock wheels and close eyes
                */
               
                // Used to get new poitns if I decide mid Race to change them
                if (lastInterpolatedPointsSetup != config->interpolatedPointsSetup)
                {
                    if (NEAR_VIEW_SETUP == config->interpolatedPointsSetup)
                    {
                        this->initPerspectiveVariables("interpolated_points_NEAR.txt");
                    }
                    else if(FAR_VIEW_SETUP == config->interpolatedPointsSetup)
                    {
                        this->initPerspectiveVariables("interpolated_points_FAR.txt");
                    }
                    else
                    {
                        this->initPerspectiveVariables("interpolated_points.txt");
                    }
                }

                uint16_t distance = getDistanceFromCarsBumper();
                std::cout << "Distance: " << distance << std::endl;
                std::cout << "CURRENT STATE: "<< config->currentState << "\n";
                if( EXITING_INTERSECTION == config->currentState)
                {
                    isBlueStatusLedOn = true;
                }
                else
                {
                    isBlueStatusLedOn = false;
                }

                if( this->isFinishLineDetected ) 
                {
                    isYellowStatusLedOn = true;
                }
                else
                {
                    isYellowStatusLedOn = false;
                }
                

                std::cout << " Angle to Left: " << finishLineLeftAngle << " degrees\n";
                std::cout << " Angle to Right: " << finishLineRightAngle << " degrees\n";

                std::cout << " (BIG LOOP) isFinishLineDetected: " << this->isFinishLineDetected << std::endl;
                if ( 1 == config->enableFinishLineDetection)
                {
                    /* WARNING: distanceBeforeIssuesAppear Is the distance where the car still sees the track and not the box in FOV*/
                    if (this->isFinishLineDetected && (distance < distanceBeforeIssuesAppear && distance > 0))
                    {
                        isObjectCloserThanDistanceBeforeIssuesAppear = true;
                    }
                    if(this->isFinishLineDetected){
                        isRadarEnabled = true;
                        if(!finishLineTimeEnabled){
                            finishLineTime = std::chrono::steady_clock::now();
                            finishLineTimeEnabled = true;
                            config->topCutOffPercentageCustomConnected = DEFAULT_AFTER_FINISH_TOP_CUTOFF_PERCENTAGE_CUSTOM_CONNECTED;
                            config->currentEdfFanSpeed = DEFAULT_EDF_FAN_AFTER_FINISH_SPEED;
                            config->corneringSpeedCoefficient = 1;
                            config->servoTurnAdjustmentCoefficient = 1;
                            config->minLookAheadInCm = 30;
                        }
                        auto now = std::chrono::steady_clock::now();
                        double elapsedMillis = std::chrono::duration<double, std::milli>(now - finishLineTime).count();
                        
                        // INFO: CAR waits for 1 seconds then stops edf   
                        if (elapsedMillis >= 500.0) {
                            isSlowDownSpeedActivated = true;
                        }         
                        if (elapsedMillis >= 1000.0) {
                            config->currentEdfFanSpeed = 0;
                        }
                    }
                    /*else if((distance > distanceBeforeIssuesAppear))
                    {
                        isObjectCloserThanDistanceBeforeIssuesAppear = false;
                    }*/
                }
                else
                {
                    /* NOTE: When modifying this bool from the menu, the car can restart the whole process */
                    config->topCutOffPercentageCustomConnected = DEFAULT_TOP_CUTOFF_PERCENTAGE_CUSTOM_CONNECTED;
                    this->isFinishLineDetected = false;
                    this->isObjectCloserThanDistanceBeforeIssuesAppear = false;
                    finishLineTimeEnabled = false;
                    isSlowDownSpeedActivated = false;
                    config->corneringSpeedCoefficient = DEFAULT_CORNERING_SPEED_COEFFICIENT;
                    config->servoTurnAdjustmentCoefficient = DEFAULT_SERVO_TURN_ADJUSTMENT_COEFFICIENT;
                }
                
                /* NOTE: this boolean is used to block the frameProcessing algorithm after the box is closer then certain distance */
                if(this->isObjectCloserThanDistanceBeforeIssuesAppear)
                {                   
                    #if 1 == ENABLE_TCP_FRAMES 
                        liveVideoFeedTCP.sendFrame(frame);
                    #endif                              
                    if(distance <= stoppingDistanceBoxLidar){
                        ppObject.setSpeed(0);
                        isRadarEnabled = false;
                    }else{
                        ppObject.setSpeed(config->minSpeedAfterFinish);
                    }

                    serialString = this->createSerialString(isBlueStatusLedOn, isYellowStatusLedOn, isRadarEnabled);
                    serial.writeToSerial(serialString);
                }
                else
                {
                    // * cv::TickMeter timer;
                    // * timer.start();
                    cv::Mat thresholdFrame = this->segmentEdges(frame);
                    rowTopCutOffThreshold = static_cast<int>(frame.rows * config->topCutOffPercentageCustomConnected); 
                    // * timer.stop();
                    // * std::cout << "segmentEdges Time: " << timer.getTimeMilli() << " ms" << std::endl;
                    
                    if ( 1 == config->enableCameraThresholdCheck ){
                        #if 1 == ENABLE_TCP_FRAMES 
                            liveVideoFeedTCP.sendFrame(frame);
                            liveVideoFeedTCP.sendFrame(thresholdFrame);
                        #endif
                    } 
                    else 
                    {
                        #if 1 != ENABLE_CAMERA_CALIBRATION 
                            
                            
                            if (config->startRace){
                                if(!startTimeEnabled){
                                    startTime = std::chrono::steady_clock::now();
                                    startTimeEnabled = true;
                                }
                                auto now = std::chrono::steady_clock::now();
                                double elapsedMillis = std::chrono::duration<double, std::milli>(now - startTime).count();

                                double totalWaitMillis = (config->waitBeforeStartSeconds + 
                                    config->waitBeforeEdfStartSeconds + config->waitBeforeFinishDetectionSeconds) * 1000.0;
                                // INFO: e.g. CAR waits for 5 seconds then starts engine
                                
                                
                                if (elapsedMillis >= (config->waitBeforeStartSeconds * 1000.0)) 
                                {
                                    config->enableCarEngine = true;
                                    config->enableCarSteering = true;
                                    ppObject.setStartSpeedOptimizedForTorque(true);
                                }
                                if (elapsedMillis >= ((config->waitBeforeStartSeconds * 1000 + config->waitBeforeEdfStartSeconds * 1000)))
                                { 
                                    ppObject.setStartSpeedOptimizedForTorque(false);
                                    config->currentEdfFanSpeed = DEFAULT_EDF_FAN_CURRENT_SPEED;
                                }
                                /* INFO: e.g. for 2 more seconds the car locks steering on 0 degrees to
                                 *       mitigate Start Line error
                                 */        
                                if (elapsedMillis >= totalWaitMillis) 
                                {
                                    config->enableFinishLineDetection = 1;
                                    this->isFinishLineDetected = false;
                                    config->startRace = false;
                                    startTimeEnabled = false;
                                }
                            }
                            else
                            {
                                startTimeEnabled = false;
                            }
                        #endif
                        
                        //this->liveVideoFeedTCP.sendFrame(thresholdFrame);
                        cv::Mat skeleton = this->skeletonizeFrame(thresholdFrame); 

                        std::vector<std::vector<cv::Point2f>> lines = this->findLines(skeleton, config->currentState, rowTopCutOffThreshold);

                        cv::Mat outputImage = cv::Mat::zeros(frame.size(), CV_8UC3);
                        
                        #if 1 == ENABLE_TCP_FRAMES 
                            this->drawLines(outputImage,lines,cv::Scalar(255, 0, 0));
                            /*for (const auto& line : lines){
                                //drawPoints(outputImage, line, cv::Scalar(255, 0, 0));
                                for (const auto& point : line){
                                    std::cout << "Point.x:" << point.x << " Point.y:" << point.y << std::endl;
                                }
                            }*/
                            //this->liveVideoFeedTCP.sendFrame(outputImage);
                        #endif
                        for (int i = 0; i < lines.size(); i++){  
                            lines[i] = this->fitPolinomial(lines[i],false);
                        }
                        
                        
                        this->drawHorizontalFromHeight(outputImage,frameHeight * config->lineStartPointY,cv::Scalar(255, 255, 255));
                        #if 1 == ENABLE_TCP_FRAMES 
                            this->drawLines(outputImage,lines,cv::Scalar(0, 0, 255));
                            for(int i = 0; i < lines.size(); ++i)
                            {
                                this->drawPoints(outputImage, lines[i], cv::Scalar(0, 255, 0));
                            }
                            this->liveVideoFeedTCP.sendFrame(outputImage);
                        #endif

                        #if 1 != ENABLE_CAMERA_CALIBRATION 
                            State state = static_cast<State>(config->currentState);
                            switch (state)
                            {
                                case FOLLOWING_LINE:
                                    this->isValidLines = true;
                                    
                                    if (lines.size() >= 1)
                                    {
                                        std::vector<std::vector<cv::Point2f>> newLines;
                                        double tempLineSize = 0;

                                        /* WARNING: This was used to counter unrecognized intersection
                                        *           The top lines that appear in an intersection are giving fals negatives
                                        */

                                        bool areAllLinesFromTheTop = true;
                                        for(int i = 0; i < lines.size(); i++){
                                             
                                            if(lines[i][0].y >= frameHeight * config->lineStartPointY){    
                                               areAllLinesFromTheTop = false;
                                            }
                                           
                                            tempLineSize = calculateLineLength(lines[i]);
                                            //std::cout << "calculateLineLength  " << i << " : " << tempLineSize << "\n";
                                            
                                            /* INFO: At start we do not detect small lines
                                             *       After finish we do not detect small lines
                                             *       Error mitigation purposes
                                             */
                                            if ( 0 == config->enableFinishLineDetection || this->isFinishLineDetected)
                                            {
                                                if( INTERSECTION_minLineLength < tempLineSize)
                                                {
                                                    newLines.push_back(lines[i]);
                                                }
                                            }
                                        }
                                        
                                        if ( 0 == config->enableFinishLineDetection)
                                        {
                                            lines.clear();
                                            lines = std::move(newLines);
                                        }
                                                                            
                                        if(!areAllLinesFromTheTop){
                                            this->isValidLines = false;
                                        }
                                    }

                                    this->isIntersection = true;
                                    for (int i = 0; i < lines.size(); i++)
                                    {
                                        if(lines[i][0].y >= frameHeight * config->lineStartPointY){
                                            this->isIntersection = false;
                                        }
                                    }
                                    if(isIntersection)
                                    {
                                        lines.clear();
                                    }
                                    
                                    //std::cout << "Line size after():" << lines.size() << "\n";
                                    //std::cout << "isIntersection:" << this->isIntersection << "\n";
                                    //std::cout << "isValidLines:" << isValidLines << "\n";
                                    if(this->isIntersection && isValidLines)
                                    {
                                        config->currentState = IN_INTERSECTION;
                                    }
                                    else
                                    {
                                        
                                        for (int i = 0; i < lines.size(); i++)
                                        {
                                            lines[i] = this->perspectiveChangeLine(lines[i], this->MatrixBirdsEyeView);
                                            if(lines[i].size() > 3) 
                                            {
                                                rdpSimplify(lines[i], config->rdp_epsilon, simplifiedLine);
                                                lines[i] = simplifiedLine;
                                            }
                                            this->is90DegreeLine = this->removeHorizontalIf90Turn(lines[i], config->currentState);
                                            if (this->is90DegreeLine)
                                            {
                                                std::vector<std::vector<cv::Point2f>> keepOnlyStraightLine;
                                                keepOnlyStraightLine.push_back(lines[i]);
                                                lines.clear();
                                                lines = std::move(keepOnlyStraightLine);
                                                break;
                                            }
                                            else
                                            {
                                                if(!this->isFinishLineDetected)
                                                {
                                                    if ( 1 == config->enableFinishLineDetection)
                                                    {
                                                        
                                                        std::cout << "Finish lines.size():" << lines.size() << "\n";
                                                        if (lines.size() >= 2) 
                                                        {
                                                            
                                                            finishLineLeftAngle = 0;
                                                            finishLineRightAngle = 0;
                                                            // Loop through all finish lines (starting from index 2)
                                                            for (size_t i = 2; i < lines.size(); ++i) {
                                                                if (lines[i].size() < 2) continue;
                                                                // Compute the finish line vector
                                                                cv::Point2f finishVec = lines[i][1] - lines[i][0];

                                                                // Compute midpoint of finish line
                                                                cv::Point2f finishMid = (lines[i][0] + lines[i][1]) * 0.5;

                                                                // Find closest segments on left and right boundaries
                                                                std::vector<cv::Point2f> leftSegment = closestSegmentOnCurve(lines[0], finishMid);
                                                                std::vector<cv::Point2f> rightSegment = closestSegmentOnCurve(lines[1], finishMid);

                                                                double distanceBetweenLines = euclideanDistance(leftSegment.front(),rightSegment.front());
                                                                double distanceFromMidPointToLeft = euclideanDistance(leftSegment.front(),finishMid);
                                                                double distanceFromMidPointToRight = euclideanDistance(rightSegment.front(),finishMid);

                                                                // FinishLine can not be outside the track
                                                                if( (distanceFromMidPointToLeft > distanceBetweenLines) || (distanceFromMidPointToRight > distanceBetweenLines))
                                                                {
                                                                    break;
                                                                }
                                                                
                                                                // Compute segment direction vectors
                                                                cv::Point2f leftVec = leftSegment[1] - leftSegment[0];
                                                                cv::Point2f rightVec = rightSegment[1] - rightSegment[0];

                                                                // Compute angles
                                                                finishLineLeftAngle = computeAngleBetweenVectors(leftVec, finishVec);
                                                                finishLineRightAngle = computeAngleBetweenVectors(rightVec, finishVec);

                                                                std::cout << "Finish Line " << (i - 1) << " Angle to Left: " << finishLineLeftAngle << " degrees\n";
                                                                std::cout << "Finish Line " << (i - 1) << " Angle to Right: " << finishLineRightAngle << " degrees\n";

                                                                
                                                                if (std::abs(finishLineLeftAngle - 90) < config->finishLineAngleRange 
                                                                    || std::abs(finishLineRightAngle - 90) < config->finishLineAngleRange) {

                                                                    std::cout << "Finish Line " << (i - 1) << " is perpendicular to at least one boundary.\n";
                                                                    if(config->enableCarSteering)
                                                                    {                                                              
                                                                        this->isFinishLineDetected = true;
                                                                        std::cout << "---------------FINISH DETECTED---------------------------\n";
                                                                    }
                                                                    break;
                                                                } else {
                                                                    std::cout << "Finish Line " << (i - 1) << " is NOT perpendicular.\n";
                                                                }

                                                                if (this->isFinishLineDetected) {
                                                                    break;  // Found it, break the outer loop too
                                                                }                                                                                          
                                                            }
                                                        }
                                                    }
                                                }
                                            }
                                        }

                                        for (int i = 0; i < lines.size(); i++)
                                        {
                                            this->extendLineToEdges(lines[i], birdsEyeViewWidth, birdsEyeViewHeight);
                                            lines[i] = this->evenlySpacePoints(lines[i], curveSamplePoints);
                                        }
                                        std::cout << "Lines.size(): " << lines.size() << "\n";
                                        
                                        #if 1 == ENABLE_TCP_FRAMES 
                                            cv::Size frameSize1(birdsEyeViewWidth, birdsEyeViewHeight);
                                            cv::Mat birdEyeViewWithPoints1 = cv::Mat::zeros(frameSize1, CV_8UC3); // 3 channels (color)
                                
                                            this->drawLines(birdEyeViewWithPoints1,lines,cv::Scalar(0, 0, 255));
                                            for(int i = 0; i < lines.size(); ++i)
                                            {
                                                this->drawPoints(birdEyeViewWithPoints1, lines[i], cv::Scalar(0, 255, 0));
                                            }
                                            //this->liveVideoFeedTCP.sendFrame(birdEyeViewWithPoints1);
                                        #endif

                                        this->getLeftRightLines(lines,this->leftLine,this->rightLine);
                                        this->allMidPoints = this->findMiddle(this->leftLine,this->rightLine,birdsEyeViewWidth,birdsEyeViewHeight);    
                                    }
                                break;
                                case IN_INTERSECTION:   
                                    if (lines.size() >= 1)
                                    {
                                        for (int i = 0; i < lines.size(); i++)
                                        {
                                            lines[i] = this->perspectiveChangeLine(lines[i], this->MatrixBirdsEyeView);
                                            if(lines[i].size() > 3) 
                                            {
                                                rdpSimplify(lines[i], config->rdp_epsilon, simplifiedLine);
                                                lines[i] = simplifiedLine;
                                            }
                                           this->is90DegreeLine = this->removeHorizontalIf90Turn(lines[i], config->currentState);
                                        }

                                        std::vector<std::vector<cv::Point2f>> newLines;
                                        double tempLineSize = 0;

                                        for(int i = 0; i < lines.size(); i++){
                                           
                                            tempLineSize = calculateLineLength(lines[i]);
                                            if( IN_INTERSECTION_minLineLength < tempLineSize)
                                            {
                                                newLines.push_back(lines[i]);
                                            }
                                        }
                                                                            
                                        lines.clear();
                                        lines = std::move(newLines);
                                        
                                    }   
                                    for (int i = 0; i < lines.size(); i++)
                                    {
                                        lines[i] = this->perspectiveChangeLine(lines[i], this->MatrixInverseBirdsEyeView);
                                    }         
                                    for (int i = 0; i < lines.size(); i++){
                                        /*  NOTE: intersection is made of 4 90degree corners
                                        *   if a line starts in the box and are exiting intersection
                                        */
                                        if(lines[i][0].y > lines[i].back().y)
                                        {
                                            if((lines[i][0].y >= (frameHeight * config->lineStartPointY))){
                                                exitingIntersectionCounter = 0;
                                                config->currentState = EXITING_INTERSECTION;
                                            }
                                        } 
                                        else
                                        {
                                            if((lines[i].back().y >= (frameHeight * config->lineStartPointY))){
                                                exitingIntersectionCounter = 0;
                                                config->currentState = EXITING_INTERSECTION;
                                            }
                                        }
                                    }
                                break;
                                case EXITING_INTERSECTION:{
                                                                        
                                    if (lines.size() < 1)
                                    {
                                        break;
                                    }
                                    
                                    for (int i = 0; i < lines.size(); i++){
                                        lines[i] = this->perspectiveChangeLine(lines[i], this->MatrixBirdsEyeView);
                                        if(lines[i].size() > 3) 
                                        {
                                            rdpSimplify(lines[i], config->rdp_epsilon, simplifiedLine);
                                            lines[i] = simplifiedLine;
                                        }
                                        this->is90DegreeLine = this->removeHorizontalIf90Turn(lines[i], config->currentState);
                                        if (this->is90DegreeLine)
                                        {
                                            std::vector<std::vector<cv::Point2f>> keepOnlyStraightLine;
                                            keepOnlyStraightLine.push_back(lines[i]);
                                            lines.clear();
                                            lines = std::move(keepOnlyStraightLine);
                                            break;
                                        }
                                    }
                                    if(!this->is90DegreeLine){
                                        exitingIntersectionCounter++;
                                        if(exitingIntersectionCounter > 10){
                                            config->currentState = FOLLOWING_LINE;
                                        }
                                    }
                                    for (int i = 0; i < lines.size(); i++)
                                    {
                                        this->extendLineToEdges(lines[i], birdsEyeViewWidth, birdsEyeViewHeight);
                                        lines[i] = this->evenlySpacePoints(lines[i], curveSamplePoints);
                                    }
                                        
                                    this->getLeftRightLines(lines,this->leftLine,this->rightLine);
                                    this->allMidPoints = this->findMiddle(this->leftLine,this->rightLine,birdsEyeViewWidth,birdsEyeViewHeight);
                                    break;
                                }
                            }

                            
                            /*
                                If car finds intersection it looks for the middle line 
                                and keeps middle till it goes past intersection.
                                Assigns middle line only once!
                            */
                        
                        #endif
                        #if 1 == ENABLE_CAMERA_CALIBRATION 
                            // TO CALIBRATE CAMERA
                            cv::Point2f result;
                            std::vector<cv::Point2f> interpolatedPoints;
                            
                            #if 1 == ENABLE_TCP_FRAMES 
                                //this->liveVideoFeedTCP.sendFrame(frame);
                            #endif
                            
                            std::cout << "frameWidth:" << this->frameWidth << "\n";
                            std::cout << "resizeFrameWidth:" << resizeFrameWidth << "\n";

                            
                            this->getLeftRightLines(lines,this->leftLine,this->rightLine);
                            this->extendLineToEdges(this->leftLine, this->frameWidth, this->frameHeight + config->distanceErrorFromChassis);
                            this->extendLineToEdges(this->rightLine, this->frameWidth, this->frameHeight + config->distanceErrorFromChassis);
                            this->allMidPoints = this->findMiddle(this->leftLine,this->rightLine,birdsEyeViewWidth,birdsEyeViewHeight);
                            
                            std::cout << "this->leftLine, calibrateTopLine" << "\n";
                            int calibrateTopLine = config->calibrateTopLinePerc * resizeFrameHeight / 100;
                            int calibrateBottomLine = config->calibrateBottomLinePerc * resizeFrameHeight / 100;
                            
                            std::cout << "calibrateTopLine:" << calibrateTopLine << "\n";
                            std::cout << "calibrateBottomLine:" << calibrateBottomLine << "\n";
                            interpolatedPoints.push_back(interpolateClosestPoints(this->leftLine, calibrateTopLine));
                            std::cout << "this->leftLine, calibrateBottomLine" << "\n";
                            interpolatedPoints.push_back(interpolateClosestPoints(this->leftLine, calibrateBottomLine));
                            std::cout << "this->rightLine, calibrateTopLine" << "\n";
                            interpolatedPoints.push_back(interpolateClosestPoints(this->rightLine, calibrateTopLine));
                            std::cout << "this->rightLine, calibrateBottomLine" << "\n";
                            interpolatedPoints.push_back(interpolateClosestPoints(this->rightLine, calibrateBottomLine));
                            std::cout << "this->allMidPoints, frameHeight" << "\n";
                            interpolatedPoints.push_back(interpolateClosestPoints(this->allMidPoints, this->frameHeight + config->distanceErrorFromChassis));
                            // Write interpolated points to a TXT file
                            
                            // Show Data on frame
                            this->writePointsToTxt(interpolatedPoints, "interpolated_points.txt");
                            this->initPerspectiveVariables("interpolated_points.txt");
                            
                            #if 1 == ENABLE_TCP_FRAMES 
                                this->drawPoints(outputImage, srcPoints, cv::Scalar(0, 255, 0));
                                this->drawHorizontalFromHeight(outputImage,calibrateTopLine,cv::Scalar(255, 255, 255));
                                this->drawHorizontalFromHeight(outputImage,calibrateBottomLine,cv::Scalar(255, 255, 255));
                                this->liveVideoFeedTCP.sendFrame(outputImage);
                            #endif
                        #else 
                        
                            cv::Size frameSize(birdsEyeViewWidth, birdsEyeViewHeight);

                            // Initialize an empty image (black by default)
                            cv::Mat birdEyeViewWithPoints = cv::Mat::zeros(frameSize, CV_8UC3); // 3 channels (color)
                            // *timer.start();
                            if(allMidPoints.empty()){
                                continue;
                            }
                            ppObject.computePurePursuit( this->allMidPoints, this->carInFramePositionBirdsEye, 
                                                    this->pixelSizeInCm, this->carTopPoint, isSlowDownSpeedActivated);
                            // * timer.stop();
                            //std::cout << "computePurePursuit Time: " << timer.getTimeMilli() << " ms" << std::endl;
                    
                            //std::cout << "pixelSizeInCm: " << this->pixelSizeInCm << std::endl;                        
                            //std::cout << "TrackCurvature radius: " << ppObject.getTrackCurvatureRadius() << std::endl;
                            //std::cout << "Calculated steeringAngleServo: " << ppObject.getSteeringAngleServo() << std::endl;
                            //std::cout << "Calculated steeringAngleDegrees: " << ppObject.getSteeringAngleDegrees() << std::endl;
                            //std::cout << "Calculated speed: " << ppObject.getSpeed() << std::endl;
                                                
                            #if 1 == ENABLE_TEENSY_SERIAL
                                serialString = this->createSerialString(isBlueStatusLedOn, isYellowStatusLedOn, isRadarEnabled);
                                serial.writeToSerial(serialString);
                            #endif
                        
                            #if 1 == ENABLE_TCP_FRAMES 
                                cv::line(birdEyeViewWithPoints, carInFramePositionBirdsEye, ppObject.getLookAheadPoint(), cv::Scalar(123, 10, 255), 2); 
                                cv::line(birdEyeViewWithPoints, carInFramePositionBirdsEye, carTopPoint, cv::Scalar(123, 10, 255), 2);  
                                
                                cv::circle(birdEyeViewWithPoints, ppObject.getLookAheadPoint(), 5, cv::Scalar(254, 34, 169), -1);
                                this->drawCircle(birdEyeViewWithPoints, carInFramePositionBirdsEye, ppObject.getLookAheadDistanceInCm()/this->pixelSizeInCm, cv::Scalar(254, 34, 169));
                                this->drawCircle(birdEyeViewWithPoints, carInFramePositionBirdsEye, (ppObject.getLookAheadDistanceInCm()+30)/this->pixelSizeInCm, cv::Scalar(0, 34, 169));
                                cv::circle(birdEyeViewWithPoints, carInFramePositionBirdsEye, 5, cv::Scalar(254, 34, 169), -1);
                            
                                this->drawPoints(birdEyeViewWithPoints, dstPoints, cv::Scalar(0, 255, 255));
                                this->drawLineVector(birdEyeViewWithPoints,leftLine,cv::Scalar(0, 255, 0));
                                this->drawLineVector(birdEyeViewWithPoints,allMidPoints,cv::Scalar(255, 255, 255));
                                this->drawLineVector(birdEyeViewWithPoints,rightLine,cv::Scalar(0, 0, 255));
                                this->drawPoints(birdEyeViewWithPoints, leftLine, cv::Scalar(0, 255, 0));
                                this->drawPoints(birdEyeViewWithPoints, rightLine, cv::Scalar(0, 0, 255));

                                //this->liveVideoFeedTCP.sendFrame(birdEyeViewWithPoints);
                                
                                std::vector<cv::Point2f> l_leftLine = this->perspectiveChangeLine(this->leftLine, MatrixInverseBirdsEyeView);
                                std::vector<cv::Point2f> l_rightLine = this->perspectiveChangeLine(this->rightLine, MatrixInverseBirdsEyeView);
                                std::vector<cv::Point2f> l_allMidPoints = this->perspectiveChangeLine(this->allMidPoints, MatrixInverseBirdsEyeView);
                                cv::Point2f l_lookAheadPoint = this->perspectiveChangePoint(ppObject.getLookAheadPoint(), MatrixInverseBirdsEyeView);

                                outputImage = cv::Mat::zeros(frame.size(),CV_8UC3);

                                
                                this->drawHorizontalFromHeight(outputImage,rowTopCutOffThreshold,cv::Scalar(0, 0, 255));
                                this->drawHorizontalFromHeight(outputImage,frameHeight * config->lineStartPointY,cv::Scalar(255, 255, 255));

                                this->drawPoints(outputImage, srcPoints, cv::Scalar(0, 255, 255));
                            
                                std::vector<cv::Point2f> horizontalLine;
                                horizontalLine.push_back(srcPoints[1]);
                                horizontalLine.push_back(srcPoints[3]);
                            
                                //this->drawLineVector(outputImage,horizontalLine,cv::Scalar(0, 255, 255));

                                cv::circle(outputImage, l_lookAheadPoint, 5, cv::Scalar(254, 34, 169), -1);
                                cv::circle(outputImage, carInFramePosition, 5, cv::Scalar(254, 34, 169), -1);
                            
                                this->drawPoints(outputImage, l_leftLine, cv::Scalar(0, 255, 0));
                                this->drawPoints(outputImage, l_rightLine, cv::Scalar(0, 0, 255));
                                this->drawLineVector(outputImage,l_leftLine,cv::Scalar(0, 255, 0));
                                this->drawLineVector(outputImage,l_allMidPoints,cv::Scalar(255, 255, 255));
                                this->drawLineVector(outputImage,l_rightLine,cv::Scalar(0, 0, 255));
                            
                                //liveVideoFeedTCP.sendFrame(outputImage);

                                cv::cvtColor(frame, frame, cv::COLOR_GRAY2BGR);

                                // Add the visualization (outputImage) on top of the original frame
                                cv::Mat overlayedImage;
                                cv::addWeighted(frame, overlayFrameWeight, outputImage, overlayFrameWeight, 0, overlayedImage);
                                liveVideoFeedTCP.sendFrame(overlayedImage);
                            #endif
                        #endif
                    }
                }
                
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

// Cuts pixel rows from image's bottom part
cv::Mat CameraProcessing::cropFrameBottom(const cv::Mat& frame, double l_bottomCutOffPercentage) {
    if (frame.rows == l_bottomCutOffPercentage) {
        return frame;
    }

    if (l_bottomCutOffPercentage < 0.0 || l_bottomCutOffPercentage >= 1.0) {
        throw std::invalid_argument("Cut Height must be between 0.0 and 1.0");
    }

    this->frameWidth = frame.cols;
    this->frameHeight = frame.rows;

    int cutOffHeight = frame.rows * l_bottomCutOffPercentage;

    int newFrameHeight = frame.rows - cutOffHeight;
    cv::Rect roi(0, 0, this->frameWidth, newFrameHeight);

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
    cv::threshold(frame, thresholdFrame, config->thresholdValue, maxThresholdValue, cv::THRESH_BINARY);
    //cv::threshold(frame, thresholdFrame, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    cv::bitwise_not(thresholdFrame, thresholdFrame);
    return thresholdFrame;
}
        
// Function to find lines in a skeletonized frame
std::vector<std::vector<cv::Point2f>> CameraProcessing::findLines(const cv::Mat& thresholdedImage, int currentState, const int rowTopCutOffThreshold){

    // Perform connected component analysis
    cv::Mat labels;
    std::vector<std::vector<cv::Point2f>> lines;
    int numLabels = customConnectedComponentsWithThreshold(thresholdedImage, labels, 6, lines, currentState, rowTopCutOffThreshold);
    
    // Return only the first 4 lines (or fewer if there are less than 4)
    if (lines.size() > 4) {
        lines.resize(4);
    }
    
    return lines;
}
        
int CameraProcessing::customConnectedComponentsWithThreshold(const cv::Mat& binaryImage, cv::Mat& labelImage,
     int radius, std::vector<std::vector<cv::Point2f>>& lines, int currentState, const int rowTopCutOffThreshold)
{
    CV_Assert(binaryImage.type() == CV_8UC1);
    labelImage = cv::Mat::zeros(binaryImage.size(), CV_32S); // Initialize label matrix
    lines.clear(); // Clear any existing lines

    int label = 1; // Start labeling from 1
    std::vector<cv::Point2f> neighborhood = generateNeighborhood(radius); // Generate dynamic neighborhood
    int pixelCount;

    if (EXITING_INTERSECTION == currentState || IN_INTERSECTION == currentState){
        for (int y = rowTopCutOffThreshold; y <= binaryImage.rows - 1; ++y) { // Top to bottom
            for (int x = 0; x < binaryImage.cols; ++x) {  // Left to right
                if (binaryImage.at<uchar>(y, x) == 255 && labelImage.at<int>(y, x) == 0) {

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

                    if (pixelCount < config->lineMinPixelCount) {
                        for (const auto& p : componentPixels) {
                            labelImage.at<int>(p.y, p.x) = 0;
                        }
                    } else {
                        std::reverse(componentPixels.begin(), componentPixels.end());
                        lines.push_back(std::move(componentPixels));
                        label++;
                    }
                }
            }
        }
    }
    else
    {
        for (int y = binaryImage.rows - 1; y >= rowTopCutOffThreshold; --y) { // Bottom to top
            for (int x = 0; x < binaryImage.cols; ++x) {  // Left to right
                if (binaryImage.at<uchar>(y, x) == 255 && labelImage.at<int>(y, x) == 0) {

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

                    if (pixelCount < config->lineMinPixelCount) {
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
        throw std::runtime_error("Line must have at least two points to extend.");
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
    if (xBottom < 0 - 0.5 * providedFrameWidth) xBottom = -0.5 * providedFrameWidth; // Manually constrain to 2 times the left edge in case line is too long
    if (xBottom >= 1.5 * providedFrameWidth) xBottom = 1.5 * providedFrameWidth; // Manually constrain to 2 times the right edge in case line is too long
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
    std::cout<< "Slope:" << slope << "\n";
    // Check if the slope is close to zero (indicating a horizontal line)
    return std::abs(slope) <= slopeThreshold;
}
// Function to remove horizontal sections if a 90-degree turn is detected
bool CameraProcessing::removeHorizontalIf90Turn(std::vector<cv::Point2f>& line, int currentState)
{
    bool has90DegreeTurn = false;
    std::vector<cv::Point2f> result;
    uint8_t index90DegreeTurn;

    double maxAngle = 0.0; // Variable to store the maximum angle

    // Check for 90-degree turns along the line
    if (line.size() == 4){
        std::cout << "ALERT:" << line << "\n";
    }
    for (int i = 1; i < line.size() - 1; ++i) 
    {
        double angle = std::abs(calculateSignedAngleThreePoints(line[i - 1], line[i], line[i + 1])* 180.0 / CV_PI);
        maxAngle = std::max(maxAngle, angle); // Update maxAngle if the current angle is larger
        //std::cout << "angle[" << i << "]: "  << angle << std::endl;

        if (std::abs(angle - 90) < config->line90DegreeAngleRange) // Close to 90 degrees
        {  
            has90DegreeTurn = true;
            index90DegreeTurn = i;
            std::cout << "Angle(has90DegreeTurn): " << angle << std::endl;
            break; // You may break early if you're only checking for 90-degree turns
        }
        
    }

    // Print the maximum angle after the loop
    //std::cout << "Maximum angle detected: " << maxAngle << " degrees" << std::endl;


    // If there is a 90-degree turn, remove the horizontal parts
    if (has90DegreeTurn) 
    {
        if (EXITING_INTERSECTION != currentState){
            for (int i = 0; i <= index90DegreeTurn; i++) 
            {
                result.push_back(line[i]);
            }
        }
        else{
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

cv::Point2f CameraProcessing::normalize(const cv::Point2f& v) {
    float len = std::sqrt(v.x * v.x + v.y * v.y);
    if (len < 1e-6) return {0, 0};
    return v * (1.0f / len);
}

cv::Point2f CameraProcessing::rightNormal(const cv::Point2f& v) {
    return {-v.y, v.x};
}

cv::Point2f CameraProcessing::leftNormal(const cv::Point2f& v) {
    return {v.y, -v.x};
}

cv::Point2f CameraProcessing:: ensureCorrectSide(const cv::Point2f& base, const cv::Point2f& offsetPoint, const cv::Point2f& historicalReference) {
    /*cv::Point2f toOffset = offsetPoint - base;
    cv::Point2f toHistory = historicalReference - base;
    if (toOffset.dot(toHistory) < 0) {
        return base - toOffset; // Flip the direction
    }*/
    return offsetPoint;
}


// Function that handles 2 Lines, 1 Line and No line cases in BirdEyeView
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


        // std::cout <<" weightedDistAtoLeft" << weightedDistAtoLeft << std::endl;
        // std::cout <<" weightedDistAtoRight" << weightedDistAtoRight << std::endl;
        // std::cout <<" weightedDistBtoLeft" << weightedDistBtoLeft << std::endl;
        // std::cout <<" weightedDistBtoRight" << weightedDistBtoRight << std::endl;
        // Check for impostors
        if ((weightedDistAtoLeft < weightedDistAtoRight && weightedDistBtoLeft < weightedDistBtoRight) || 
            (weightedDistAtoRight < weightedDistAtoLeft && weightedDistBtoRight < weightedDistBtoLeft)) 
        {
            // std::cout << "Distances: " << std::endl;
            // std::cout << "weightedDistAtoLeft: " << weightedDistAtoLeft << std::endl;
            // std::cout << "weightedDistAtoRight: " << weightedDistAtoRight << std::endl;
            // std::cout << "weightedDistBtoLeft: " << weightedDistBtoLeft << std::endl;
            // std::cout << "weightedDistBtoRight: " << weightedDistBtoRight << std::endl;

            double smallestDistance = weightedDistAtoLeft;
            std::string smallestType = "AtoLeft";

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

            // std::cout << "Smallest distance: " << smallestDistance << " (" << smallestType << ")" << std::endl;
            
            // std::cout << "config->trackLaneWidthOffset:" << config->trackLaneWidthOffset << std::endl;
            // std::cout << "with pixelsize:" << (config->trackLaneWidthOffset * pixelSizeInCm) << std::endl;
            if (smallestType == "AtoLeft") {
                // Line A is closest to the left
                // std::cout << "Classifying Line A as the left line." << std::endl;
                leftFitted = lines[0];
                rightFitted.clear();
                for (const auto& point : leftFitted) {
                    shiftedPoint = cv::Point2f(point.x + trackLaneWidthInPixel + (config->trackLaneWidthOffset * pixelSizeInCm), point.y); // Mirror left to right
                    rightFitted.push_back(shiftedPoint);
                }
                firstPointLeftLine = firstPointLineA; // Update historical left point
                firstPointRightLine = rightFitted[0]; // Update historical right point
            } else if (smallestType == "AtoRight") {
                // Line A is closest to the right
                rightFitted = lines[0];
                leftFitted.clear();
                for (const auto& point : rightFitted) {
                    shiftedPoint = cv::Point2f(point.x - trackLaneWidthInPixel - (config->trackLaneWidthOffset * pixelSizeInCm), point.y); // Mirror right to left
                    leftFitted.push_back(shiftedPoint);
                }
                firstPointRightLine = firstPointLineA; // Update historical right point
                firstPointLeftLine = leftFitted[0]; // Update historical left point
            } else if (smallestType == "BtoLeft") {
                // Line B is closest to the left
                leftFitted = lines[1];
                rightFitted.clear();
                for (const auto& point : leftFitted) {
                    shiftedPoint = cv::Point2f(point.x + trackLaneWidthInPixel + (config->trackLaneWidthOffset * pixelSizeInCm), point.y); // Mirror left to right
                    rightFitted.push_back(shiftedPoint);
                }
                firstPointLeftLine = firstPointLineB; // Update historical left point
                firstPointRightLine = rightFitted[0]; // Update historical right point
            } else if (smallestType == "BtoRight") {
                // Line B is closest to the right
                rightFitted = lines[1];
                leftFitted.clear();
                for (const auto& point : rightFitted) {
                    shiftedPoint = cv::Point2f(point.x - trackLaneWidthInPixel - (config->trackLaneWidthOffset * pixelSizeInCm), point.y); // Mirror right to left
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
    else if (1 == lines.size())
    {
        // Get the points of the line
        cv::Point2f pointFront(lines[0][0].x, lines[0][0].y);       // First point of the line

        double weightedDistToLeft;
        double weightedDistToRight;

        if((undefinedPoint == firstPointLeftLine) && (undefinedPoint == firstPointRightLine))
        {
            // NOTE: Check on first iteration if the line is on the left of middle or on the right
            if (pointFront.x < (birdsEyeViewWidth / 2))
            {
                weightedDistToLeft = 0;
                weightedDistToRight = 1;        
            } 
            else
            {
                weightedDistToLeft = 1;
                weightedDistToRight = 0;
            }
        }
        else{
            // Weighted distance comparison using the first point
            weightedDistToLeft = euclideanDistance(pointFront, firstPointLeftLine) 
                                    + alpha * std::abs(pointFront.y - firstPointLeftLine.y);
            weightedDistToRight = euclideanDistance(pointFront, firstPointRightLine) 
                                        + alpha * std::abs(pointFront.y - firstPointRightLine.y);
        }
        // Output debug information
        //std::cout << "Weighted Distance to Left: " << weightedDistToLeft << std::endl;
        //std::cout << "Weighted Distance to Right: " << weightedDistToRight << std::endl;

        /*// Classify the line as left or right based on weighted distances
        if (weightedDistToLeft < weightedDistToRight) {
            // Single line is closer to historical left, so it is the left line
            leftFitted = lines[0];
            rightFitted.clear();
            for (size_t i = 0; i < leftFitted.size() - 1; ++i) {
                // Compute the midpoint of each segment
                float mid_x = (leftFitted[i].x + leftFitted[i + 1].x) / 2.0;
                float mid_y = (leftFitted[i].y + leftFitted[i + 1].y) / 2.0;

                // Compute direction vector
                float dx = leftFitted[i + 1].x - leftFitted[i].x;
                float dy = leftFitted[i + 1].y - leftFitted[i].y;
                float length = std::sqrt(dx * dx + dy * dy);

                // Compute perpendicular offset (normalized)
                float offset_x = (trackLaneWidthInPixel + config->trackLaneWidthOffset) * (-dy / length);
                float offset_y = (trackLaneWidthInPixel + config->trackLaneWidthOffset) * (dx / length);

                // Compute the mirrored point
                shiftedPoint = cv::Point2f(mid_x + offset_x, mid_y + offset_y);
                rightFitted.push_back(shiftedPoint);
            }


            firstPointLeftLine = pointFront; // Update historical left point
            firstPointRightLine = rightFitted[0]; // Update historical right point
        } else {
            // Single line is closer to historical right, so it is the right line
            rightFitted = lines[0];
            leftFitted.clear();
            for (size_t i = 0; i < rightFitted.size() - 1; ++i) {
                // Compute the midpoint of each segment
                float mid_x = (rightFitted[i].x + rightFitted[i + 1].x) / 2.0;
                float mid_y = (rightFitted[i].y + rightFitted[i + 1].y) / 2.0;

                // Compute direction vector
                float dx = rightFitted[i + 1].x - rightFitted[i].x;
                float dy = rightFitted[i + 1].y - rightFitted[i].y;
                float length = std::sqrt(dx * dx + dy * dy);

                // Compute perpendicular offset (normalized)
                float offset_x = (trackLaneWidthInPixel + config->trackLaneWidthOffset) * (dy / length);
                float offset_y = (trackLaneWidthInPixel + config->trackLaneWidthOffset) * (-dx / length);

                // Compute the mirrored point
                shiftedPoint = cv::Point2f(mid_x + offset_x, mid_y + offset_y);
                leftFitted.push_back(shiftedPoint);
            }

            
            firstPointRightLine = pointFront; // Update historical right point
            firstPointLeftLine = leftFitted[0]; // Update historical left point
        }*/
        // Classify the line as left or right based on weighted distances
        if(weightedDistToLeft < weightedDistToRight)
        {
            std::cout << "Mirroring Right" << "\n";
        }else{
            std::cout << "Mirroring Left" << "\n";
        }
        if (weightedDistToLeft < weightedDistToRight) {
            // Single line is closer to historical left, so it is the left line
            leftFitted = lines[0];
            rightFitted.clear();
            
            // First point
            cv::Point2f dir0 = normalize(leftFitted[1] - leftFitted[0]);
            cv::Point2f firstOffset = leftFitted[0] + rightNormal(dir0) * (trackLaneWidthInPixel + config->trackLaneWidthOffset);
            rightFitted.push_back(ensureCorrectSide(leftFitted[0], firstOffset, firstPointRightLine));


            // Middle points
            for (size_t i = 1; i < leftFitted.size() - 1; ++i) {
                cv::Point2f v1 = normalize(leftFitted[i] - leftFitted[i - 1]);
                cv::Point2f v2 = normalize(leftFitted[i + 1] - leftFitted[i]);
                cv::Point2f bisector = normalize(v1 + v2);

                cv::Point2f normalR = (std::abs(bisector.x) < 1e-6 && std::abs(bisector.y) < 1e-6) ? rightNormal(v1) : rightNormal(bisector);
                cv::Point2f offsetPoint = leftFitted[i] + normalR * (trackLaneWidthInPixel + config->trackLaneWidthOffset);
                rightFitted.push_back(ensureCorrectSide(leftFitted[i], offsetPoint, rightFitted.back()));
            }

             // Last point
            size_t last = leftFitted.size() - 1;
            cv::Point2f dirN = normalize(leftFitted[last] - leftFitted[last - 1]);
            cv::Point2f lastOffset = leftFitted[last] + rightNormal(dirN) * (trackLaneWidthInPixel + config->trackLaneWidthOffset);
            rightFitted.push_back(ensureCorrectSide(leftFitted[last], lastOffset, rightFitted.back()));


            firstPointLeftLine = pointFront; // Update historical left point
            firstPointRightLine = rightFitted[0]; // Update historical right point
        } else {
            // Single line is closer to historical right, so it is the right line
            rightFitted = lines[0];
            leftFitted.clear();

            // First point
            cv::Point2f dir0 = normalize(rightFitted[1] - rightFitted[0]);
            cv::Point2f firstOffset = rightFitted[0] + leftNormal(dir0) * (trackLaneWidthInPixel + config->trackLaneWidthOffset);
            leftFitted.push_back(ensureCorrectSide(rightFitted[0], firstOffset, firstPointLeftLine));

             // Middle points
            for (size_t i = 1; i < rightFitted.size() - 1; ++i) {
                cv::Point2f v1 = normalize(rightFitted[i] - rightFitted[i - 1]);
                cv::Point2f v2 = normalize(rightFitted[i + 1] - rightFitted[i]);
                cv::Point2f bisector = normalize(v1 + v2);

                cv::Point2f normalL = (std::abs(bisector.x) < 1e-6 && std::abs(bisector.y) < 1e-6) ? leftNormal(v1) : leftNormal(bisector);
                cv::Point2f offsetPoint = rightFitted[i] + normalL * (trackLaneWidthInPixel + config->trackLaneWidthOffset);
                leftFitted.push_back(ensureCorrectSide(rightFitted[i], offsetPoint, leftFitted.back()));
            }

            
            // Last point
            size_t last = rightFitted.size() - 1;
            cv::Point2f dirN = normalize(rightFitted[last] - rightFitted[last - 1]);
            cv::Point2f lastOffset = rightFitted[last] + leftNormal(dirN) * (trackLaneWidthInPixel + config->trackLaneWidthOffset);
            leftFitted.push_back(ensureCorrectSide(rightFitted[last], lastOffset, leftFitted.back()));

            
            firstPointRightLine = pointFront; // Update historical right point
            firstPointLeftLine = leftFitted[0]; // Update historical left point
        }
    }

    else 
    {
    //firstPointLeftLine = undefinedPoint;
    //firstPointRightLine = undefinedPoint;

        // TBD NO LINES, STOP CAR
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
}
std::vector<cv::Point2f> CameraProcessing::findMiddle(std::vector<cv::Point2f>& leftLine, std::vector<cv::Point2f>& rightLine, int providedFrameWidth, int providedFrameHeight) {
    std::vector<cv::Point2f> midPoints;
    if( leftLine.size() < 2 || rightLine.size() < 2) {
        throw std::runtime_error("Both leftLine and rightLine must have at least two points.");
    }

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

// Function that adds up segments size
double CameraProcessing::calculateLineLength(const std::vector<cv::Point2f>& curve){
    double size = 0;
    for (int i = 1; i < curve.size(); i++){
        size += euclideanDistance(curve[i-1],curve[i]);
    }
    return size;
}
uint16_t CameraProcessing::getMedian(uint16_t *array, int size) {
  // Simple bubble sort
  for (int i = 0; i < size - 1; i++) {
    for (int j = i + 1; j < size; j++) {
      if (array[j] < array[i]) {
        unsigned long temp = array[i];
        array[i] = array[j];
        array[j] = temp;
      }
    }
  }
  
  // Return middle element
  if (size % 2 == 1) {
    return array[size / 2];
  } else {
    int mid = size / 2;
    return (array[mid - 1] + array[mid]) / 2;
  }
}
// Functions to read Specific Data From Recieved Data via Serial
uint16_t CameraProcessing::getDistanceFromCarsBumper(){
    static int8_t bufferIndex = 0;
    static uint16_t localBuffer[distanceMedianFilterSampleSize] = {0};  

    uint16_t value = 0;
    std::string receivedData = serial.getReceivedData();
    size_t pos = receivedData.find(';');

    std::cout << "receivedData value: " << receivedData << std::endl;
    if (pos != std::string::npos) {
        value = std::stoi(receivedData.substr(pos + 1));
        //std::cout << "Extracted value: " << value << std::endl;
    }
    
    localBuffer[bufferIndex] = value;
    bufferIndex = (bufferIndex + 1) % distanceMedianFilterSampleSize;
    uint16_t medianPeriod = getMedian(localBuffer, distanceMedianFilterSampleSize);

    /*  WARNING: Prevent underflow*/
    return (medianPeriod > config->distanceSensorError) ? 
           (medianPeriod - config->distanceSensorError) : 0; 
}


// Perspective transform related methods
void CameraProcessing::initPerspectiveVariables(std::string inputTxt) {
    auto loadedPoints = readPointsFromTxt(inputTxt);

    this->srcPoints = {
        cv::Point2f(loadedPoints[0].x, loadedPoints[0].y),
        cv::Point2f(loadedPoints[1].x, loadedPoints[1].y),
        cv::Point2f(loadedPoints[2].x, loadedPoints[2].y),
        cv::Point2f(loadedPoints[3].x, loadedPoints[3].y)
    };

    dstPoints = {
        cv::Point2f(
            birdsEyeViewWidth / 2 - widthDstPoints / 2,
            birdsEyeViewHeight - heightDstPoints - config->distanceErrorFromChassis
        ), // Top-left corner
        cv::Point2f(
            birdsEyeViewWidth / 2 - widthDstPoints / 2,
            birdsEyeViewHeight - config->distanceErrorFromChassis
        ), // Bottom-left corner
        cv::Point2f(
            birdsEyeViewWidth / 2 + widthDstPoints / 2,
            birdsEyeViewHeight - heightDstPoints - config->distanceErrorFromChassis
        ), // Top-right corner
        cv::Point2f(
            birdsEyeViewWidth / 2 + widthDstPoints / 2,
            birdsEyeViewHeight - config->distanceErrorFromChassis
        ) // Bottom-right corner
    };

    this->trackLaneWidthInPixel = euclideanDistance(dstPoints[3], dstPoints[1]) ;//+ 20;   //Error mitigation +10
    this->trackLaneWidthInCm = trackWidthInCm;
    this->pixelSizeInCm = trackLaneWidthInCm / trackLaneWidthInPixel;
    // std::cout << "trackLaneWidthInCm: " << trackLaneWidthInCm << std::endl;
    // std::cout << "trackLaneWidthInPixel: " << trackLaneWidthInPixel << std::endl;
    // std::cout << "pixelSizeInCm: " << pixelSizeInCm << std::endl;
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
    if (!line.empty()) {
        for (int i = 1; i < line.size(); i++) {
            cv::line(frame, line[i-1], line[i], color, 2);  // Green
        }
    }
}

void CameraProcessing::drawLines(cv::Mat& frame, const std::vector<std::vector<cv::Point2f>> lines, const cv::Scalar& color)
{
    if (!lines.empty()) {
        for (int i = 0; i < lines.size(); ++i) {
            drawLineVector(frame,lines[i],color);
        }
    }
}

void CameraProcessing::drawCircle(cv::Mat& image, const cv::Point2f& center, int radius, const cv::Scalar& color, int thickness)
{
    cv::circle(image, center, radius, color, thickness);
}

std::string  CameraProcessing::createSerialString(bool isBlueStatusLedOn, bool isYellowStatusLedOn, bool isRadarEnabled)
{
    std::string serialString = "";
    int checksum = 0;

    if (config->enableCarEngine){
        serialString += "1;";
    }else{
        serialString += "0;";
    }
    if (config->enableCarSteering){
        //serialString += std::to_string(ppObject.getSteeringAngleServo()) + ";";
        //if(IN_INTERSECTION == config->currentState)
        if(false)
        {
            serialString += "0;";
        }
        else
        {
            serialString += to_string_with_precision(ppObject.getSteeringAngleServo(),2) + ";";
        }
    }else{
        serialString +="0;";
    }
    
    // WARNING: Used to not build up PID ERROR
    if (config->enableCarEngine){
        serialString += to_string_with_precision(ppObject.getSpeed(),2) + ";";
    }
    else
    {
        serialString += "0;";
    }

    if (this->isFinishLineDetected){
        serialString +="1;";
    }else{
        serialString +="0;";
    }
    serialString += std::to_string(distanceBeforeIssuesAppear) + ";";

    if (isRadarEnabled){
        serialString +="1;";
    }else{
        serialString +="0;";
    }

    if(isBlueStatusLedOn)
    {
        serialString +="1;";
    }
    else
    {
        serialString +="0;";
    }

    if(isYellowStatusLedOn)
    {
        serialString +="1;";
    }
    else
    {
        serialString +="0;";
    }

    serialString += std::to_string(config->currentEdfFanSpeed);

    
    for (char c : serialString) {
        checksum += static_cast<unsigned char>(c);
    }

    serialString += ";" + std::to_string(checksum);
    
    return serialString;
}
