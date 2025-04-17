#include "SerialPortFunctions/SerialPort.hpp"

void debix::SerialPort::initializeSerialPort(const std::string& portName) {
    try {
        serialPort.Open(portName);
        serialPort.SetBaudRate(LibSerial::BaudRate::BAUD_230400);
        serialPort.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
        serialPort.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
        serialPort.SetParity(LibSerial::Parity::PARITY_NONE);
        std::cout << "Serial port initialized successfully.\n";
    } catch (const std::exception &e) {
        std::cerr << "Error initializing serial port: " << e.what() << std::endl;
        exit(1);
    }
}

/*  Expected format: "<enableCarEngine>;<servoAngle>;<speed>;<isFinishLineDetected>;
 *  //<distanceBeforeIssuesAppear>;<stoppingDistanceBoxLidar>;<blueStatusLed>;<yellowStatusLed>;<afterFinishLineSpeed>"
 *  // Example: "1;10.5;150.6;1;60;10;1,1,50"
 */
void debix::SerialPort::writeToSerial(const std::string &data) {
    std::lock_guard<std::mutex> lock(serialMutex);
    try {
        int fd = serialPort.GetFileDescriptor();
        if (!writeAllWithTimeout(fd, data + "\n", 100)) {
            std::cerr << "[Serial] Write timed out.\n";
        } else {
            std::cout << "[TX] Sent: " << data << std::endl;
        }
    } catch (const std::exception &e) {
        std::cerr << "Error writing to serial port: " << e.what() << std::endl;
    }
}


void debix::SerialPort::readFromSerial() {
    while (this->running) {
        try {
            if (serialPort.IsDataAvailable()) {
                // ActualSpeed; DistanceFromLidarInCm
                std::string localReceivedData;
                serialPort.ReadLine(localReceivedData, '\n', 50);
                if (!localReceivedData.empty()) {
                    this->receivedData = localReceivedData;
                    //std::cout << "[RX] Received: " << receivedData;
                }
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            } 
        } catch (const std::exception &e) {
            std::cerr << "Error at reading: " << e.what() << std::endl;
        }
    }
}

void debix::SerialPort::startSerialRead() {
    this->running = true;
    this->serialReadThread = std::thread(&debix::SerialPort::readFromSerial, this);
} 

void debix::SerialPort::stopSerialRead() {
    if (this->running) {
        this->running = false;
        if (this->serialReadThread.joinable()) {
            this->serialReadThread.join();  
        }
    }
}


void debix::SerialPort::connectTeensy(const std::string& portName) {
    this->initializeSerialPort(portName);
    this->startSerialRead();
}