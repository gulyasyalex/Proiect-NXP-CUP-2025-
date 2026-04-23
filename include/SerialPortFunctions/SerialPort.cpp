#include "SerialPortFunctions/SerialPort.hpp"

bool debix::SerialPort::initializeSerialPort(const std::string& portName) {
    try {
        serialPort.Open(portName);
        serialPort.SetBaudRate(LibSerial::BaudRate::BAUD_230400);
        serialPort.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
        serialPort.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
        serialPort.SetParity(LibSerial::Parity::PARITY_NONE);
        std::cout << "Serial port initialized successfully.\n";
    } catch (const std::exception &e) {
        std::cerr << "Error initializing serial port: " << e.what() << std::endl;
        return false;
    }
    return true;
}

/*  Expected format: "<enableCarEngine>;<servoAngle>;<speed>;<isFinishLineDetected>;
 *  //<distanceBeforeIssuesAppear>;<isRadarEnabled>;<blueStatusLed>;<yellowStatusLed>;"
 *  // Example: "1;10.5;150.6;1;60;1;1,1"
 */
void debix::SerialPort::writeToSerial(const std::string &data) {
    std::lock_guard<std::mutex> lock(serialMutex);
    try {
        int fd = serialPort.GetFileDescriptor();
        if (!writeAllWithTimeout(fd, data + "\n", 100)) {
            std::cerr << "[Serial] Write timed out.\n";
        } else {
            //std::cout << "[TX] Sent: " << data << std::endl;
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


bool debix::SerialPort::connectTeensy(const std::string& portName) {
    if(!this->initializeSerialPort(portName))
    {
        return false;
    }
    this->startSerialRead();
    return true;
}

bool debix::SerialPort::disconnectTeensy() {
    this->stopSerialRead();

    std::lock_guard<std::mutex> lock(serialMutex);

    try {
        if (serialPort.IsOpen()) {
            serialPort.Close();
            std::cout << "Serial port disconnected successfully.\n";
            return true;
        }
    } catch (const std::exception &e) {
        std::cerr << "Error disconnecting serial port: " << e.what() << std::endl;
        return false;
    }
    return false;
}