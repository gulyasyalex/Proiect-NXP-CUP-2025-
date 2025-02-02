#ifndef OLED_DISPLAY_HPP
#define OLED_DISPLAY_HPP
/*
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

class OledDisplay {
private:
    int file;                 // File descriptor for I2C device
    const char* i2cBus;       // I2C bus path
    uint32_t oledAddress;     // OLED I2C address

    void initializeOLED() {
        writeCommand(0xAE); // Display OFF
        writeCommand(0xD5); // Set display clock divide ratio
        writeCommand(0x80);
        writeCommand(0xA8); // Set multiplex ratio
        writeCommand(OLED_HEIGHT - 1);
        writeCommand(0xD3); // Set display offset
        writeCommand(0x00);
        writeCommand(0x40); // Set start line address
        writeCommand(0x8D); // Enable charge pump
        writeCommand(0x14);
        writeCommand(0x20); // Set memory addressing mode
        writeCommand(0x00); // Horizontal addressing mode
        writeCommand(0xA1); // Set segment re-map
        writeCommand(0xC8); // Set COM output scan direction
        writeCommand(0xDA); // Set COM pins hardware configuration
        writeCommand(0x12);
        writeCommand(0x81); // Set contrast control
        writeCommand(0x7F);
        writeCommand(0xD9); // Set pre-charge period
        writeCommand(0xF1);
        writeCommand(0xDB); // Set VCOMH deselect level
        writeCommand(0x20);
        writeCommand(0xA4); // Entire display ON
        writeCommand(0xA6); // Set normal display
        writeCommand(0xAF); // Display ON
    }
public:
    // OLED Display Parameters
    static constexpr int OLED_WIDTH = 128;
    static constexpr int OLED_HEIGHT = 64;
    static constexpr uint8_t OLED_CMD = 0x00;
    static constexpr uint8_t OLED_DATA = 0x40;

    OledDisplay(const char* bus = "/dev/i2c-5", uint32_t address = 0x3C)
        : file(-1), i2cBus(bus), oledAddress(address) {}

    ~OledDisplay() {
        if (file >= 0) {
            close(file); // Close I2C file descriptor
        }
    }

    bool initialize() {
        // Open the I2C device
        file = open(i2cBus, O_RDWR);
        if (file < 0) {
            std::cerr << "Failed to open I2C bus: " << i2cBus << std::endl;
            return false;
        }

        // Set the I2C address for the OLED
        if (ioctl(file, I2C_SLAVE, oledAddress) < 0) {
            std::cerr << "Failed to set I2C address: 0x" << std::hex << oledAddress << std::endl;
            return false;
        }

        // Initialize the OLED
        initializeOLED();
        return true;
    }

    void writeCommand(uint8_t command) {
        uint8_t buffer[2] = {OLED_CMD, command};
        if (write(file, buffer, 2) != 2) {
            std::cerr << "Failed to write command to OLED" << std::endl;
        }
    }


    void clearDisplay() {
        for (int page = 0; page < 8; page++) {
            writeCommand(0xB0 + page); // Set page start address
            writeCommand(0x00);       // Set lower column start address
            writeCommand(0x10);       // Set higher column start address

            for (int col = 0; col < OLED_WIDTH; col++) {
                uint8_t buffer[2] = {OLED_DATA, 0x00}; // Clear each pixel
                if (write(file, buffer, 2) != 2) {
                    std::cerr << "Failed to write data to OLED" << std::endl;
                }
            }
        }
    }

    void drawText(const std::string& text) {
        writeCommand(0xB0); // Set page address to 0
        writeCommand(0x00); // Set lower column start address
        writeCommand(0x10); // Set higher column start address

        for (char c : text) {
            uint8_t buffer[2] = {OLED_DATA, static_cast<uint8_t>(c)};
            if (write(file, buffer, 2) != 2) {
                std::cerr << "Failed to write text data to OLED" << std::endl;
            }
        }
    }
};
*/
#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <iostream>
#include <cstring>

// SSD1306 Display Constants
#define I2C_BUS "/dev/i2c-5" // Adjust based on your I2C bus
#define SSD1306_ADDR 0x3C    // Default I2C address of the SSD1306
#define OLED_WIDTH 128
#define OLED_HEIGHT 32

// I2C File Descriptor
int i2c_fd = -1;

// Write a command to the OLED
void writeCommand(uint8_t command) {
    uint8_t buffer[2] = {0x00, command}; // 0x00: Command identifier
    if (write(i2c_fd, buffer, 2) != 2) {
        std::cerr << "Failed to write command to OLED" << std::endl;
    }
}

// Initialize the SSD1306 OLED
void initializeOLED() {
    writeCommand(0xAE); // Display OFF
    writeCommand(0xD5); // Set display clock divide ratio
    writeCommand(0x80); // Suggested ratio
    writeCommand(0xA8); // Set multiplex ratio
    writeCommand(0x3F); // Height - 1
    writeCommand(0xD3); // Set display offset
    writeCommand(0x00);
    writeCommand(0x40); // Set start line address
    writeCommand(0x8D); // Enable charge pump
    writeCommand(0x14);
    writeCommand(0x20); // Set memory addressing mode
    writeCommand(0x00); // Horizontal addressing mode
    writeCommand(0xA1); // Set segment re-map
    writeCommand(0xC8); // Set COM output scan direction
    writeCommand(0xDA); // Set COM pins hardware configuration
    writeCommand(0x12);
    writeCommand(0x81); // Set contrast control
    writeCommand(0x7F); // Maximum contrast
    writeCommand(0xD9); // Set pre-charge period
    writeCommand(0xF1);
    writeCommand(0xDB); // Set VCOMH deselect level
    writeCommand(0x40);
    writeCommand(0xA4); // Disable entire display ON
    writeCommand(0xA6); // Normal display (not inverted)
    writeCommand(0xAF); // Display ON
}

// Clear the OLED display
void clearDisplay() {
    for (int page = 0; page < 8; ++page) {
        writeCommand(0xB0 + page); // Set page address
        writeCommand(0x00);       // Set lower column start address
        writeCommand(0x10);       // Set higher column start address

        uint8_t buffer[OLED_WIDTH + 1];
        buffer[0] = 0x40; // Data identifier
        memset(buffer + 1, 0x00, OLED_WIDTH); // Clear each page

        if (write(i2c_fd, buffer, OLED_WIDTH + 1) != OLED_WIDTH + 1) {
            std::cerr << "Failed to clear OLED display" << std::endl;
        }
    }
}

// Display text using a simple font
void displayText(const std::string& text) {
    const uint8_t font5x7[][5] = {
        {0x00, 0x00, 0x00, 0x00, 0x00}, // Space
        {0x00, 0x00, 0x5F, 0x00, 0x00}, // !
        {0x00, 0x07, 0x00, 0x07, 0x00}, // "
        {0x14, 0x7F, 0x14, 0x7F, 0x14}, // #
        // Add additional characters as needed
    };

    writeCommand(0xB0); // Start at page 0
    writeCommand(0x00); // Lower column start address
    writeCommand(0x10); // Higher column start address

    for (char c : text) {
        if (c < 32 || c > 127) c = 32; // Replace unsupported chars with space

        uint8_t buffer[6] = {0x40}; // Data identifier
        memcpy(buffer + 1, font5x7[c - 32], 5);
        buffer[6] = 0x00; // Add space between characters

        if (write(i2c_fd, buffer, sizeof(buffer)) != sizeof(buffer)) {
            std::cerr << "Failed to write character to OLED" << std::endl;
        }
    }
}


#endif
