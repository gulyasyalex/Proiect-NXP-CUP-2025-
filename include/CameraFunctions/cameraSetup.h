#ifndef CAMERA_SETUP_H
#define CAMERA_SETUP_H

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include "config.h"
#include "mathFunctions.h"
#include <unistd.h>
#include <iostream>
#include <vector>
#include <fstream>

/*
interpolated_points/txt
53 20
-20 130
224 20
285 130

63 20
-13 140
233 20
306 140

69 20
-60 140
200 20
314 140

62 45
-77 170
247 45
360 170
158 175
*/

void printWorkingDirectory();
std::string getCameraIndex();
void printWorkingDirectory() {
    char cwd[1024];
    if (getcwd(cwd, sizeof(cwd)) != nullptr) {
        std::cout << "Current Working Directory: " << cwd << std::endl;
    } else {
        std::cerr << "Error getting current working directory." << std::endl;
    }
}
// Function to execute the shell script and capture the output
std::string getCameraIndex() {
    // Path to the shell script
    std::string scriptPath = "./include/CameraFunctions/find_camera_index.sh";
    char buffer[128];
    std::string result = "";

    // Open a pipe to run the shell script
    FILE* pipe = popen(scriptPath.c_str(), "r");
    if (!pipe) {
        std::cerr << "Error: Unable to open pipe to script." << std::endl;
        return "-1";
    }

    // Read the output of the script
    while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
        result += buffer;
    }

    // Close the pipe
    pclose(pipe);

    // Remove any trailing newline character
    result.erase(result.find_last_not_of(" \n\r\t") + 1);

    return result;
}

#endif 