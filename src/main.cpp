#include <iostream>
#include <thread>
#include <vector>
#include <memory>
#include <csignal>
#include <chrono>
#include "SerialPortFunctions/SerialPort.hpp"
#include "CameraFunctions/CameraProcessing.hpp"
#include "CameraFunctions/cameraSetup.h"
#include "OledLcdFunctions/OledDisplay.hpp"
#include "config.h"
#include "u8g2/csrc/u8g2.h"

// Global shared pointer for managing the camera
std::shared_ptr<CameraProcessing> global_camera;
debix::SerialPort& serial = debix::SerialPort::getInstance();

void signalHandler(int signal) {
    std::cout << "\nReceived signal " << signal << ". Stopping camera..." << std::endl;

    if (global_camera) {
        global_camera->stopCapture();
        global_camera->stopFrameProcessing();
    }
    #if 1 == ENABLE_TEENSY_SERIAL
        serial.stopSerialRead();
    #endif

    std::exit(signal);
}

int main() {
    std::signal(SIGINT, signalHandler); // Register SIGINT handler

    #if 1 == ENABLE_TEENSY_SERIAL
        serial.connectTeensy(SERIAL_PORT);
    #endif

    
    // Open the I2C device for OLED LCD
    i2c_fd = open(I2C_BUS, O_RDWR);
    if (i2c_fd < 0) {
        std::cerr << "Failed to open I2C bus: " << I2C_BUS << std::endl;
        return -1;
    }

    // Set the I2C address for the OLED LCD
    if (ioctl(i2c_fd, I2C_SLAVE, SSD1306_ADDR) < 0) {
        std::cerr << "Failed to set I2C address: 0x" << std::hex << SSD1306_ADDR << std::endl;
        return -1;
    }
    

    // for (int i = 0; i < 3; ++i) {
    //     serial.writeToSerial("Hello from Debix, message " + std::to_string(i));
    //     std::this_thread::sleep_for(std::chrono::seconds(1));
    // }

    try {
        // Initialize the global camera instance
        global_camera = std::make_shared<CameraProcessing>();

        #if 1 == ENABLE_CAMERA_STREAMING
            // Get the camera index from the shell scriptsc
            std::string cameraIndexStr = getCameraIndex();
            int cameraIndex = std::stoi(cameraIndexStr);
            std::cout << "cameraIndex: " << cameraIndex << std::endl;
            global_camera->setParameters(cameraIndex, captureFrameWidth, captureFrameHeight, captureFps);
        #endif

        global_camera->startCapture();
        global_camera->startFrameProcessing();

        // oled.clearDisplay();
        // oled.drawText("Camera running...");
        
        while (global_camera->isRunning()) {
            //std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        global_camera->stopCapture();
        global_camera->stopFrameProcessing();

    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}
