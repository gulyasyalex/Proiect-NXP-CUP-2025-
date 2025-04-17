#ifndef SERIAL_PORT_F_HPP
#define SERIAL_PORT_F_HPP

#include <libserial/SerialPort.h>
#include <iostream>
#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>
#include "WriteAllWithTimeout.hpp"

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
#endif
