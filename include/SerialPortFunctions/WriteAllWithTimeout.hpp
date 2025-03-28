#ifndef WRITE_ALL_WITH_TIMEOUT_HPP
#define WRITE_ALL_WITH_TIMEOUT_HPP

#include <unistd.h>
#include <fcntl.h>
#include <sys/select.h>
#include <string>
#include <chrono>
#include <thread>
#include <iostream>
#include <cstring>
#include <cerrno>

// Inline to avoid multiple definition errors across translation units
inline bool writeAllWithTimeout(int fd, const std::string& data, int timeoutMs = 100) {
    const char* buffer = data.c_str();
    size_t total_written = 0;
    size_t to_write = data.size();

    auto start_time = std::chrono::steady_clock::now();

    while (total_written < to_write) {
        auto now = std::chrono::steady_clock::now();
        int elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();
        if (elapsed > timeoutMs) {
            std::cerr << "[Serial] Write timeout! Wrote " << total_written
                      << "/" << to_write << " bytes.\n";
            return false;
        }

        fd_set write_fds;
        FD_ZERO(&write_fds);
        FD_SET(fd, &write_fds);

        struct timeval tv = {0, 5000}; // 5ms wait

        int ready = select(fd + 1, nullptr, &write_fds, nullptr, &tv);
        if (ready > 0) {
            ssize_t written = write(fd, buffer + total_written, to_write - total_written);
            if (written > 0) {
                total_written += written;
            } else if (errno != EAGAIN && errno != EWOULDBLOCK) {
                std::cerr << "[Serial] Write error: " << strerror(errno) << "\n";
                return false;
            }
        } else if (ready < 0) {
            std::cerr << "[Serial] select() error: " << strerror(errno) << "\n";
            return false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    return true;
}

#endif // WRITE_ALL_WITH_TIMEOUT_HPP
