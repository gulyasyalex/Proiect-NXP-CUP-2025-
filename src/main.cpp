#include <iostream>
#include <thread>
#include <vector>
#include <memory>
#include <csignal>
#include <chrono>
#include "SerialPortFunctions/SerialPort.hpp"
#include "CameraFunctions/CameraProcessing.hpp"
#include "CameraFunctions/cameraSetup.h"
#include "config.h"

// Global shared pointer for managing the camera
std::shared_ptr<CameraProcessing> global_camera;
debix::SerialPort& serial = debix::SerialPort::getInstance();

void signalHandler(int signal) {
    std::cout << "\nReceived signal " << signal << ". Stopping camera..." << std::endl;

    if (global_camera) {
        global_camera->stopCapture();
        global_camera->stopFrameProcessing();
    }
    serial.stopSerialRead();

    std::exit(signal);
}

int main() {
    std::signal(SIGINT, signalHandler); // Register SIGINT handler

    serial.initializeSerialPort(SERIAL_PORT);

    serial.startSerialRead();

    for (int i = 0; i < 10; ++i) {
        serial.writeToSerial("Hello from Debix, message " + std::to_string(i));
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }


    try {
        // Initialize the global camera instance
        global_camera = std::make_shared<CameraProcessing>();

        #if 1 == ENABLE_STREAMING
            // Get the camera index from the shell scriptsc
            std::string cameraIndexStr = getCameraIndex();
            int cameraIndex = std::stoi(cameraIndexStr);
            std::cout << "cameraIndex: " << cameraIndex << std::endl;
            global_camera->setParameters(cameraIndex, captureFrameWidth, captureFrameHeight, captureFps);
        #endif

        global_camera->startCapture();
        global_camera->startFrameProcessing();

        while (global_camera->isRunning()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        global_camera->stopCapture();
        global_camera->stopFrameProcessing();

    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}
