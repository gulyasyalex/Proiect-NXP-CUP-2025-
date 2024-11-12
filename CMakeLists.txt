cmake_minimum_required(VERSION 3.5)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

project("DEBIX")

# Find OpenCV package (no need to set OpenCV_DIR explicitly on Linux if it's installed correctly)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)



# Include directories from OpenCV
include_directories(
    ${LIBUSB_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/include
    /home/debix/pixy2/src/host/libpixyusb2/include
    /usr/local/include
    #/usr/local/include
    #/home/debix/Documents/pixy2/common/inc
    #/usr/include/libusb-1.0
)

# Add .cpp files found in src
aux_source_directory(${CMAKE_SOURCE_DIR}/src SRC_FILES)

# Create an executable
add_executable(DEBIX ${SRC_FILES})
# add_executable(DEBIX src/main.cpp)

# Link OpenCV libraries
find_package(CURL REQUIRED)
link_directories(${LIBUSB_LIBRARY_DIRS})

target_link_libraries(DEBIX ${LIBUSB_LIBRARIES} ${OpenCV_LIBS} ${CURL_LIBRARIES} gdal netcdf 
    /home/debix/pixy2/build/libpixyusb2/libpixy2.a)

    find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)
include_directories(${LIBUSB_INCLUDE_DIRS})
link_directories(${LIBUSB_LIBRARY_DIRS})
target_link_libraries(DEBIX ${LIBUSB_LIBRARIES})
