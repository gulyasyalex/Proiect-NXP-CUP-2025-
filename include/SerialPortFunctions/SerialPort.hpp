#ifndef SERIAL_PORT_F_HPP
#define SERIAL_PORT_F_HPP

#include <libserial/SerialPort.h>
#include <iostream>
#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>

namespace debix{
    class SerialPort{
        private:
            std::atomic<bool> running{false};
            LibSerial::SerialPort serialPort;
            std::mutex serialMutex;
            std::thread serialReadThread;
            std::string receivedData;

            SerialPort() = default;
            ~SerialPort() 
            { 
                stopSerialRead(); 
                if (serialPort.IsOpen())
                    serialPort.Close(); 
            }

            // Delete copy constructor and assignment operator
            SerialPort(const SerialPort&) = delete;
            SerialPort& operator=(const SerialPort&) = delete;

        public:
            // Static method to get the single instance
            static SerialPort& getInstance() {
                static SerialPort instance;  // Static local instance
                return instance;
            }
            void initializeSerialPort(const std::string& portName);
            void writeToSerial(const std::string &data);
            void readFromSerial();
            void startSerialRead();
            void stopSerialRead();
            void connectTeensy(const std::string& portName) ;
            std::string getReceivedData(){
                return this->receivedData;
            };
    };
}

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

void debix::SerialPort::writeToSerial(const std::string &data) {
    std::lock_guard<std::mutex> lock(serialMutex);  // Ensure thread safety
    try {
        serialPort.Write(data + "\n");  // Append newline for easier reading
        std::cout << "[TX] Sent: " << data << std::endl;
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
#endif
