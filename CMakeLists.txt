cmake_minimum_required(VERSION 3.5)
project("DEBIX")

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE Debug)

# Find packages
find_package(PkgConfig REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CURL REQUIRED)
pkg_check_modules(GPIOD REQUIRED libgpiod)
#pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

# Include directories
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    #${LIBUSB_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/include
    /usr/local/include
    /usr/include
    #/usr/local/include
    #/usr/include/libusb-1.0
)

# Collect source files from src/ and include/
file(GLOB_RECURSE SRC_FILES
    ${CMAKE_SOURCE_DIR}/src/*.cpp
    ${CMAKE_SOURCE_DIR}/include/**/*.cpp
)

# Create executable
add_executable(DEBIX ${SRC_FILES} ${CMAKE_SOURCE_DIR}/src/main.cpp)

# Link libraries
target_link_libraries(DEBIX 
    ${OpenCV_LIBS} 
    ${CURL_LIBRARIES}
    #${LIBUSB_LIBRARIES}  
    serial
    gpiod
    )

# link_directories(${LIBUSB_LIBRARY_DIRS})

message(STATUS "Configured DEBIX project with ${CMAKE_CXX_STANDARD} and ${CMAKE_BUILD_TYPE} mode.")


