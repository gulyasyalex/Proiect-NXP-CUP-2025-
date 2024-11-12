#include "libpixyusb2.h"
#include <opencv2/opencv.hpp>

Pixy2 pixy;

int demosaic(uint16_t width, uint16_t height, const uint8_t *bayerImage, uint32_t *image)
{
    uint32_t x, y, xx, yy, r, g, b;
    uint8_t *pixel0, *pixel;

    for (y = 0; y < height; y++)
    {
        yy = y;
        if (yy == 0)
            yy++;
        else if (yy == height - 1)
            yy--;
        pixel0 = (uint8_t *)bayerImage + yy * width;
        for (x = 0; x < width; x++, image++)
        {
            xx = x;
            if (xx == 0)
                xx++;
            else if (xx == width - 1)
                xx--;
            pixel = pixel0 + xx;
            if (yy & 1)
            {
                if (xx & 1)
                {
                    r = *pixel;
                    g = (*(pixel - 1) + *(pixel + 1) + *(pixel + width) + *(pixel - width)) >> 2;
                    b = (*(pixel - width - 1) + *(pixel - width + 1) + *(pixel + width - 1) + *(pixel + width + 1)) >> 2;
                }
                else
                {
                    r = (*(pixel - 1) + *(pixel + 1)) >> 1;
                    g = *pixel;
                    b = (*(pixel - width) + *(pixel + width)) >> 1;
                }
            }
            else
            {
                if (xx & 1)
                {
                    r = (*(pixel - width) + *(pixel + width)) >> 1;
                    g = *pixel;
                    b = (*(pixel - 1) + *(pixel + 1)) >> 1;
                }
                else
                {
                    r = (*(pixel - width - 1) + *(pixel - width + 1) + *(pixel + width - 1) + *(pixel + width + 1)) >> 2;
                    g = (*(pixel - 1) + *(pixel + 1) + *(pixel + width) + *(pixel - width)) >> 2;
                    b = *pixel;
                }
            }
            *image = (b << 16) | (g << 8) | r;
        }
    }
    return 0;
}

int main()
{
    int Result;
    uint8_t *bayerFrame;
    uint32_t rgbFrame[PIXY2_RAW_FRAME_WIDTH * PIXY2_RAW_FRAME_HEIGHT];

    printf("Connecting to Pixy2...");

    // Initialize Pixy2 Connection
    Result = pixy.init();
    if (Result < 0)
    {
        printf("Error: pixy.init() returned %d\n", Result);
        return Result;
    }
    printf("Success\n");

    // Get Pixy2 Version information
    Result = pixy.getVersion();
    if (Result < 0)
    {
        printf("Error: pixy.getVersion() returned %d\n", Result);
        return Result;
    }
    pixy.version->print();

    pixy.m_link.stop();

    while (true)
    {
        // Grab raw frame in BGGR Bayer format, 1 byte per pixel
        Result = pixy.m_link.getRawFrame(&bayerFrame);
        if (Result < 0)
        {
            printf("Error: pixy.getRawFrame() returned %d\n", Result);
            break;
        }

        // Convert Bayer frame to RGB frame
        demosaic(PIXY2_RAW_FRAME_WIDTH, PIXY2_RAW_FRAME_HEIGHT, bayerFrame, rgbFrame);

        // Create an OpenCV Mat with RGB data
        cv::Mat frame(PIXY2_RAW_FRAME_HEIGHT, PIXY2_RAW_FRAME_WIDTH, CV_8UC3);

        // Copy the RGB frame data into the OpenCV Mat
        for (int y = 0; y < PIXY2_RAW_FRAME_HEIGHT; ++y)
        {
            for (int x = 0; x < PIXY2_RAW_FRAME_WIDTH; ++x)
            {
                uint32_t pixel = rgbFrame[y * PIXY2_RAW_FRAME_WIDTH + x];
                frame.at<cv::Vec3b>(y, x) = cv::Vec3b(pixel & 0xFF, (pixel >> 8) & 0xFF, (pixel >> 16) & 0xFF);
            }
        }

        // Display the frame
        cv::imshow("Pixy2 Raw Image", frame);

        // Exit on pressing 'q'
        if (cv::waitKey(1) == 'q')
            break;
    }

    // Resume Pixy2 processing
    pixy.m_link.resume();
    return 0;
}
