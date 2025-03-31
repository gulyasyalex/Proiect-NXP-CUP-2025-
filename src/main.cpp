#include <iostream>
#include <thread>
#include <vector>
#include <memory>
#include <csignal>
#include <chrono>

// Needed for Shared Memory access
#include <fcntl.h>                                  // Needed for O_CREAT, O_RDWR flags in shm_open()
#include <sys/mman.h>                               // Required for mmap(), shm_open(), and munmap()
#include <unistd.h>                                 // Required for ftruncate(), sleep(), and close()

#include "SerialPortFunctions/SerialPort.hpp"
#include "CameraFunctions/CameraProcessing.hpp"
#include "CameraFunctions/cameraSetup.h"
#include "config.h"

#include <sys/wait.h>
#include <gpiod.h>

#include <cstddef>  // Required for `offsetof`


std::atomic<pid_t> child_pid(-1);  

// Global shared pointer for managing the camera
std::shared_ptr<CameraProcessing> global_camera;
std::shared_ptr<SharedConfig> global_config;
debix::SerialPort& serial = debix::SerialPort::getInstance();

void signalHandler(int signal);
void checkButtonPress();

int main() {
    std::signal(SIGINT, signalHandler); // Register SIGINT handler

    #if 1 == ENABLE_TEENSY_SERIAL
        serial.connectTeensy(SERIAL_PORT);
    #endif    
    // for (int i = 0; i < 3; ++i) {
    //     serial.writeToSerial("Hello from Debix, message " + std::to_string(i));
    //     std::this_thread::sleep_for(std::chrono::seconds(1));
    // }

    // Shared memory for LCD and Rotary python code
    int shm_fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
    if ( -1 == shm_fd ){
        std::cerr << "Function shm_open failed!" << std::endl;
        return 1;
    }


    if (ftruncate(shm_fd, SHM_SIZE) == -1) {
        std::cerr << "ftruncate failed!" << std::endl;
        close(shm_fd);
        return 1;
    }

    void* mapped_mem = mmap(NULL, SHM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (mapped_mem == MAP_FAILED) {
        std::cerr << "Function mmap failed!" << std::endl;
        close(shm_fd);
        return 1;
    }

    global_config = std::shared_ptr<SharedConfig>(static_cast<SharedConfig*>(mapped_mem), [](SharedConfig* ptr) {
        munmap(ptr, SHM_SIZE);  // Free shared memory when `shared_ptr` is destroyed
    });

    memset(global_config.get(), 0, SHM_SIZE);

    // Initialize values
    global_config->startRace = DEFAULT_START_RACE;
    global_config->enableCarEngine = DEFAULT_ENABLE_CAR_ENGINE;
    global_config->enableCarSteering = DEFAULT_ENABLE_CAR_STEERING;
    global_config->enableCameraThresholdCheck = ENABLE_CAMERA_THRESHOLD_CHECK;
    global_config->enableFinishLineDetection = ENABLE_FINISH_LINE_DETECTION;
    global_config->thresholdValue = DEFAULT_THRESHOLD_VALUE;
    global_config->distanceErrorFromChassis = DEFAULT_DISTANCE_ERROR_FROM_CHASSIS;
    global_config->lineMinPixelCount = _minLinePixelCount;
    global_config->distanceSensorError = DEFAULT_DISTANCE_FROM_SENSOR_ERROR;
    global_config->stoppingDistanceBoxFrontEnd = DEFAULT_STOPPING_DISTANCE_BOX_FRONT_END;

    global_config->calibrateTopLinePerc = DEFAULT_CALIBRATE_TOP_LINE;
    global_config->calibrateBottomLinePerc = DEFAULT_CALIBRATE_BOTTOM_LINE;
    global_config->trackLaneWidthOffset = DEFAULT_TRACK_LANE_WIDTH_OFFSET;
    global_config->topImageCutPercentage = DEFAULT_TOP_IMAGE_CUT_PERCENTAGE;
    global_config->bottomImageCutPercentage = DEFAULT_BOTTOM_IMAGE_CUT_PERCENTAGE;
    global_config->topCutOffPercentageCustomConnected = DEFAULT_TOP_CUTOFF_PERCENTAGE_CUSTOM_CONNECTED;
    global_config->line90DegreeAngleRange = DEFAULT_LINE_90_DEGREE_ANGLE_RANGE;
    global_config->finishLineAngleRange = DEFAULT_FINISH_LINE_ANGLE_RANGE;
    global_config->servoTurnAdjustmentCoefficient = DEFAULT_SERVO_TURN_ADJUSTMENT_COEFFICIENT;
    global_config->corneringSpeedCoefficient = DEFAULT_CORNERING_SPEED_COEFFICIENT;
    global_config->minSpeed = DEFAULT_MIN_SPEED;
    global_config->maxSpeed = DEFAULT_MAX_SPEED;
    global_config->curvatureFactor = DEFAULT_CURVATURE_FACTOR;
    global_config->k_min = DEFAULT_K_MIN;
    global_config->k_max = DEFAULT_K_MAX;
    global_config->R_minInCm = DEFAULT_R_MIN_IN_CM;
    global_config->R_maxInCm = DEFAULT_R_MAX_IN_CM;
    global_config->minLookAheadInCm = DEFAULT_MIN_LOOKAHEAD_IN_CM;
    global_config->maxLookAheadInCm = DEFAULT_MAX_LOOKAHEAD_IN_CM;
    global_config->waitBeforeStartSeconds = DEFAULT_WAIT_BEFORE_START_SECONDS;                      
    global_config->straightWheelTimerSeconds = DEFAULT_STRAIGHT_WHEEL_TIMER_SECONDS;                   

    std::cout << "Shared memory initialized with defaults: " << std::endl;

    SharedConfig* raw_config = global_config.get();  // Get raw pointer from shared_ptr

    std::cout << "\nIntegers: ";
    std::cout << raw_config->startRace << " "
            << raw_config->enableCarEngine << " "
            << raw_config->enableCarSteering << " "
            << raw_config->enableCameraThresholdCheck << " "
            << raw_config->enableFinishLineDetection << " "
            << raw_config->thresholdValue << " "
            << raw_config->distanceErrorFromChassis << " "
            << raw_config->lineMinPixelCount << " "
            << raw_config->distanceSensorError << " "
            << raw_config->stoppingDistanceBoxFrontEnd << std::endl;

    std::cout << "Doubles: ";
    std::cout << raw_config->calibrateTopLinePerc << " "
            << raw_config->calibrateBottomLinePerc << " " << " "
            << raw_config->trackLaneWidthOffset << " "
            << raw_config->topImageCutPercentage << " "
            << raw_config->bottomImageCutPercentage << " "
            << raw_config->topCutOffPercentageCustomConnected << " "
            << raw_config->line90DegreeAngleRange << " "
            << raw_config->finishLineAngleRange << " "
            << raw_config->servoTurnAdjustmentCoefficient << " "
            << raw_config->corneringSpeedCoefficient << " "
            << raw_config->minSpeed << " "
            << raw_config->maxSpeed << " "
            << raw_config->curvatureFactor << " "
            << raw_config->k_min << " "
            << raw_config->k_max << " "
            << raw_config->R_minInCm << " "
            << raw_config->R_maxInCm << " "
            << raw_config->minLookAheadInCm << " "
            << raw_config->maxLookAheadInCm << " "
            << raw_config->waitBeforeStartSeconds << " "
            << raw_config->straightWheelTimerSeconds << std::endl;

    std::cout << "sizeof(SharedConfig): " << sizeof(SharedConfig) << " bytes" << std::endl;

    std::cout << "Offsets in SharedConfig:" << std::endl;
    std::cout << "startRace: " << offsetof(SharedConfig, startRace) << std::endl;
    std::cout << "enableCarEngine: " << offsetof(SharedConfig, enableCarEngine) << std::endl;
    std::cout << "enableCarSteering: " << offsetof(SharedConfig, enableCarSteering) << std::endl;
    std::cout << "enableCameraThresholdCheck: " << offsetof(SharedConfig, enableCameraThresholdCheck) << std::endl;
    std::cout << "enableFinishLineDetection: " << offsetof(SharedConfig, enableFinishLineDetection) << std::endl;
    std::cout << "thresholdValue: " << offsetof(SharedConfig, thresholdValue) << std::endl;
    std::cout << "distanceErrorFromChassis: " << offsetof(SharedConfig, distanceErrorFromChassis) << std::endl;
    std::cout << "lineMinPixelCount: " << offsetof(SharedConfig, lineMinPixelCount) << std::endl;
    std::cout << "distanceSensorError: " << offsetof(SharedConfig, distanceSensorError) << std::endl;
    std::cout << "stoppingDistanceBoxFrontEnd: " << offsetof(SharedConfig, stoppingDistanceBoxFrontEnd) << std::endl;

    std::cout << "calibrateTopLinePerc: " << offsetof(SharedConfig, calibrateTopLinePerc) << std::endl;
    std::cout << "calibrateBottomLinePerc: " << offsetof(SharedConfig, calibrateBottomLinePerc) << std::endl;
    std::cout << "trackLaneWidthOffset: " << offsetof(SharedConfig, trackLaneWidthOffset) << std::endl;
    std::cout << "topImageCutPercentage: " << offsetof(SharedConfig, topImageCutPercentage) << std::endl;
    std::cout << "bottomImageCutPercentage: " << offsetof(SharedConfig, bottomImageCutPercentage) << std::endl;
    std::cout << "topCutOffPercentageCustomConnected: " << offsetof(SharedConfig, topCutOffPercentageCustomConnected) << std::endl;
    std::cout << "line90DegreeAngleRange: " << offsetof(SharedConfig, line90DegreeAngleRange) << std::endl;
    std::cout << "finishLineAngleRange: " << offsetof(SharedConfig, finishLineAngleRange) << std::endl;
    std::cout << "servoTurnAdjustmentCoefficient: " << offsetof(SharedConfig, servoTurnAdjustmentCoefficient) << std::endl;
    std::cout << "corneringSpeedCoefficient: " << offsetof(SharedConfig, corneringSpeedCoefficient) << std::endl;
    std::cout << "minSpeed: " << offsetof(SharedConfig, minSpeed) << std::endl;
    std::cout << "maxSpeed: " << offsetof(SharedConfig, maxSpeed) << std::endl;
    std::cout << "curvatureFactor: " << offsetof(SharedConfig, curvatureFactor) << std::endl;
    std::cout << "k_min: " << offsetof(SharedConfig, k_min) << std::endl;
    std::cout << "k_max: " << offsetof(SharedConfig, k_max) << std::endl;
    std::cout << "R_minInCm: " << offsetof(SharedConfig, R_minInCm) << std::endl;
    std::cout << "R_maxInCm: " << offsetof(SharedConfig, R_maxInCm) << std::endl;
    std::cout << "minLookAheadInCm: " << offsetof(SharedConfig, minLookAheadInCm) << std::endl;
    std::cout << "maxLookAheadInCm: " << offsetof(SharedConfig, maxLookAheadInCm) << std::endl;
    std::cout << "waitBeforeStartSeconds: " << offsetof(SharedConfig, waitBeforeStartSeconds) << std::endl;
    std::cout << "straightWheelTimerSeconds: " << offsetof(SharedConfig, straightWheelTimerSeconds) << std::endl;

    std::cout << "sizeof(SharedConfig): " << sizeof(SharedConfig) << " bytes" << std::endl;

    std::thread buttonThread(checkButtonPress);

    try {

        #if 1 == ENABLE_CAMERA_STREAMING
            // Get the camera index from the shell scriptsc
            std::string cameraIndexStr = getCameraIndex();
            int cameraIndex = std::stoi(cameraIndexStr);
            std::cout << "cameraIndex: " << cameraIndex << std::endl;
            // Initialize the global camera instance
            global_camera = std::make_shared<CameraProcessing>(cameraIndex, captureFrameWidth, captureFrameHeight, captureFps, global_config);
        #else
        
            // Initialize the global camera instance
            global_camera = std::make_shared<CameraProcessing>(global_config);
        #endif

        global_camera->startCapture();
        global_camera->startFrameProcessing();

        
        while (global_camera->isRunning()) {
            //std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        if (buttonThread.joinable()) {
            buttonThread.join();
        }

    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}

void signalHandler(int signal) {
    std::cout << "\nReceived signal " << signal << ". Stopping camera..." << std::endl;

    if (global_camera) {
        global_camera->stopCapture();
        global_camera->stopFrameProcessing();
    }
    #if 1 == ENABLE_TEENSY_SERIAL
        std::string serialString = "0;0;0;0;0;0";
        serial.writeToSerial(serialString);
        serial.stopSerialRead();
    #endif

    std::exit(signal);
}

void stopChildProcess() {
    if (child_pid > 0) {
        std::cout << "Stopping Python process (PID: " << child_pid.load() << ")" << std::endl;
        kill(child_pid, SIGTERM);  // Graceful stop
        waitpid(child_pid, NULL, 0);  // Cleanup zombie process
        child_pid = -1;  // Reset PID to allow restarting
    }
}

void checkButtonPress() {
    // ✅ Keep track of GPIO initialization
    gpiod_chip *chip = nullptr;
    gpiod_line *line = nullptr;

    // ✅ Open GPIO only once at the start
    chip = gpiod_chip_open("/dev/gpiochip0");
    if (!chip) {
        std::cerr << "Failed to open GPIO chip!" << std::endl;
        return;
    }

    line = gpiod_chip_get_line(chip, SW_LINE);
    if (!line) {
        std::cerr << "Failed to get GPIO line!" << std::endl;
        gpiod_chip_close(chip);
        return;
    }

    if (gpiod_line_request_input(line, "button_reader") < 0) {
        std::cerr << "Failed to set GPIO as input!" << std::endl;
        gpiod_chip_close(chip);
        return;
    }

    while (true) {
        if(line != nullptr){
            int button_state = gpiod_line_get_value(line);
        
            if (button_state == 0 && child_pid.load() == -1) {  // Button pressed & no process running
                std::cout << "Button pressed! Starting menu_rotary.py..." << std::endl;

                // ✅ Release GPIO before starting Python
                gpiod_line_release(line);
                gpiod_chip_close(chip);
                chip = nullptr;
                line = nullptr;

                pid_t pid = fork();

                if (pid == 0) {  // Child process
                    execlp("python3", "python3", "menu_rotary.py", NULL);
                    std::cerr << "Failed to execute menu_rotary.py" << std::endl;
                    exit(1);
                } else if (pid > 0) {
                    child_pid = pid;  // Store new child PID
                }
            }
        }

        // ✅ Check if Python exited and reinitialize GPIO
        if (child_pid.load() > 0) {
            int status;
            pid_t result = waitpid(child_pid.load(), &status, WNOHANG);
            if (result > 0) {
                std::cout << "Python process (PID: " << child_pid.load() << ") has exited. Cleaning up." << std::endl;
                child_pid = -1;  // Reset child process ID

                // ✅ Reinitialize GPIO after Python exits
                chip = gpiod_chip_open("/dev/gpiochip0");
                if (!chip) {
                    std::cerr << "Failed to open GPIO chip!" << std::endl;
                    return;
                }

                line = gpiod_chip_get_line(chip, SW_LINE);
                if (!line) {
                    std::cerr << "Failed to get GPIO line!" << std::endl;
                    gpiod_chip_close(chip);
                    return;
                }

                if (gpiod_line_request_input(line, "button_reader") < 0) {
                    std::cerr << "Failed to set GPIO as input!" << std::endl;
                    gpiod_chip_close(chip);
                    return;
                }
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(200));  // Check button every 200ms
    }

    // ✅ Proper cleanup before exiting
    gpiod_line_release(line);
    gpiod_chip_close(chip);
}

// void checkButtonPress() {
//     gpiod_chip *chip = gpiod_chip_open("/dev/gpiochip0");
//     if (!chip) {
//         std::cerr << "Failed to open GPIO chip!" << std::endl;
//         return;
//     }

//     gpiod_line *line = gpiod_chip_get_line(chip, SW_LINE);
//     if (!line) {
//         std::cerr << "Failed to get GPIO line!" << std::endl;
//         return;
//     }

//     if (gpiod_line_request_input(line, "button_reader") < 0) {
//         std::cerr << "Failed to set GPIO as input!" << std::endl;
//         return;
//     }

//     while (true) {
//         int button_state = gpiod_line_get_value(line);

//         if (button_state == 1 && child_pid.load() == -1) {  // Button pressed & no process running
//             std::cout << "Button pressed! Starting menu_rotary.py..." << std::endl;
            
//             gpiod_line_release(line);
//             gpiod_chip_close(chip);
            
//             pid_t pid = fork();

//             if (pid == 0) {  // Child process
//                 execlp("python3", "python3", "menu_rotary.py", NULL);
//                 std::cerr << "Failed to execute menu_rotary.py" << std::endl;
//                 exit(1);
//             } else if (pid > 0) {
//                 child_pid = pid;  // Store new child PID
//             }
//         }

//         // Check if Python exited naturally & clean up
//         if (child_pid.load() > 0) {
//             int status;
//             pid_t result = waitpid(child_pid.load(), &status, WNOHANG);
//             if (result > 0) {
//                 std::cout << "Python process (PID: " << child_pid.load() << ") has exited. Cleaning up." << std::endl;
//                 child_pid = -1;  // Reset child process ID
//                     gpiod_chip *chip = gpiod_chip_open("/dev/gpiochip0");
//                     if (!chip) {
//                         std::cerr << "Failed to open GPIO chip!" << std::endl;
//                         return;
//                     }

//                     gpiod_line *line = gpiod_chip_get_line(chip, SW_LINE);
//                     if (!line) {
//                         std::cerr << "Failed to get GPIO line!" << std::endl;
//                         return;
//                     }

//                     if (gpiod_line_request_input(line, "button_reader") < 0) {
//                         std::cerr << "Failed to set GPIO as input!" << std::endl;
//                         return;
//                     }
//             }
//         }

//         std::this_thread::sleep_for(std::chrono::milliseconds(200));  // Check button every 200ms
//     }

//     gpiod_line_release(line);
//     gpiod_chip_close(chip);
//}
