#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <iostream>

class LinuxI2C {
private:
    int file;
    const char* i2cBus;
    uint8_t deviceAddr;

public:
    LinuxI2C(const char* bus, uint8_t address)
        : i2cBus(bus), deviceAddr(address), file(-1) {}

    ~LinuxI2C() {
        if (file >= 0) {
            close(file);
        }
    }

    bool begin() {
        file = open(i2cBus, O_RDWR);
        if (file < 0) {
            std::cerr << "Failed to open I2C bus: " << i2cBus << std::endl;
            return false;
        }

        if (ioctl(file, I2C_SLAVE, deviceAddr) < 0) {
            std::cerr << "Failed to set I2C address: 0x" << std::hex << deviceAddr << std::endl;
            return false;
        }

        return true;
    }

    void writeCommand(uint8_t command) {
        uint8_t buffer[2] = {0x00, command};
        if (write(file, buffer, 2) != 2) {
            std::cerr << "Failed to send command: 0x" << std::hex << int(command) << std::endl;
        }
    }

    void writeData(uint8_t* data, size_t size) {
        uint8_t buffer[size + 1];
        buffer[0] = 0x40; // Control byte for data
        memcpy(buffer + 1, data, size);

        if (write(file, buffer, size + 1) != size + 1) {
            std::cerr << "Failed to send data" << std::endl;
        }
    }
};
