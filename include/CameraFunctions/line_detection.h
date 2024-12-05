#ifndef LINE_DETECTION_H
#define LINE_DETECTION_H

#include <opencv2/opencv.hpp>
#include "config.h"
#include "thinning.h"

cv::Point referencePoint(frameWidth / 2, frameHeight);

double lineLength(const cv::Vec4i& line);
double euclidianDistance(cv::Point p1, cv::Point p2);
cv::Point calculateMidpoint(const cv::Point& p1, const cv::Point& p2);
cv::Point calculateCentroid(const std::vector<cv::Point>& line);
cv::Mat skeletonizeFrame(const cv::Mat& thresholdedImage);
void drawPoints(cv::Mat& image, const std::vector<cv::Point2f>& points, const cv::Scalar& color);
void extendLineToEdges(std::vector<cv::Point>& middleLine);
std::vector<std::vector<cv::Point>> findLines(const cv::Mat& thresholdedImage);
std::vector<cv::Point> smoothPoints(const std::vector<cv::Point>& points, int windowSize);
std::vector<std::vector<cv::Point>> fitPolinomial(const cv::Mat& frame, std::vector<std::vector<cv::Point>> &lines);
std::vector<cv::Point> evenlySpacePoints(const std::vector<cv::Point>& line, int num_points);
std::vector<cv::Point> findMiddle(std::vector<cv::Point>& leftFitted, std::vector<cv::Point>& rightFitted);

// Function to calculate the length of a line
double lineLength(const cv::Vec4i& line) {
    int x1 = line[0], y1 = line[1], x2 = line[2], y2 = line[3];
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}
// Function to calculate the Euclidean distance between two points
double euclidianDistance(cv::Point p1, cv::Point p2) {
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}
// Function to calculate the angle between three points
double calculateAngle(cv::Point2f p1, cv::Point2f p2, cv::Point2f p3) {
    cv::Point2f vec1 = p1 - p2;
    cv::Point2f vec2 = p3 - p2;
    double dotProduct = vec1.x * vec2.x + vec1.y * vec2.y;
    double mag1 = std::sqrt(vec1.x * vec1.x + vec1.y * vec1.y);
    double mag2 = std::sqrt(vec2.x * vec2.x + vec2.y * vec2.y);
    return std::acos(dotProduct / (mag1 * mag2)) * 180.0 / CV_PI;
}
// Function to calculate the midpoint between two points
cv::Point calculateMidpoint(const cv::Point& p1, const cv::Point& p2) {
    return cv::Point((p1.x + p2.x) / 2, (p1.y + p2.y) / 2);
}
// Function to calculate the centroid of a line (average of the points)
cv::Point calculateCentroid(const std::vector<cv::Point>& line) {
    cv::Point centroid(0, 0);
    for (const auto& point : line) {
        centroid.x += point.x;
        centroid.y += point.y;
    }
    centroid.x /= line.size();
    centroid.y /= line.size();
    return centroid;
}
// Function to calculate Euclidean distance
double euclideanDistanceCoord(int x1, int y1, int x2, int y2) {
    return std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}
std::vector<cv::Point> generateNeighborhood(int radius) {
    std::vector<cv::Point> neighbors;
    for (int dy = -radius; dy <= radius; ++dy) {
        for (int dx = -radius; dx <= radius; ++dx) {
            if (std::sqrt(dx * dx + dy * dy) <= radius) { // Ensure point lies within the circle
                neighbors.emplace_back(dx, dy);
            }
        }
    }
    return neighbors;
}

int customConnectedComponentsWithThreshold(const cv::Mat& binaryImage, cv::Mat& labelImage, int radius, std::vector<std::vector<cv::Point>>& lines) {
    
    // This algorithm start from bottom left

    CV_Assert(binaryImage.type() == CV_8UC1);
    labelImage = cv::Mat::zeros(binaryImage.size(), CV_32S); // Initialize label matrix
    lines.clear(); // Clear any existing lines

    int label = 1; // Start labeling from 1
    const int minPixelCount = 10;
    const int rowThreshold = static_cast<int>(binaryImage.rows * 0.4); // Top 40% cutoff
    std::vector<cv::Point> neighborhood = generateNeighborhood(radius); // Generate dynamic neighborhood

    // Start iterating from the bottom-left corner
    for (int y = binaryImage.rows - 1; y >= 0; --y) { // Bottom to top
        for (int x = 0; x < binaryImage.cols; ++x) {  // Left to right
            if (binaryImage.at<uchar>(y, x) == 255 && labelImage.at<int>(y, x) == 0) {
                // If within the top 40% of the image, skip this component
                if (y <= rowThreshold) {
                    continue;
                }

                // Start a new connected component
                std::queue<cv::Point> queue;
                queue.push(cv::Point(x, y));
                labelImage.at<int>(y, x) = label;

                int pixelCount = 0; // Initialize pixel count for this label
                std::vector<cv::Point> componentPixels; // Store pixel locations for this component

                while (!queue.empty()) {
                    cv::Point p = queue.front();
                    queue.pop();
                    pixelCount++;
                    componentPixels.push_back(p);

                    for (const auto& offset : neighborhood) {
                        int nx = p.x + offset.x;
                        int ny = p.y + offset.y;

                        if (nx >= 0 && ny >= 0 && nx < binaryImage.cols && ny < binaryImage.rows) {
                            if (binaryImage.at<uchar>(ny, nx) == 255 && labelImage.at<int>(ny, nx) == 0) {
                                labelImage.at<int>(ny, nx) = label;
                                queue.push(cv::Point(nx, ny));
                            }
                        }
                    }
                }

                // If the component is too small, discard it
                if (pixelCount < minPixelCount) {
                    for (const auto& p : componentPixels) {
                        labelImage.at<int>(p.y, p.x) = 0; // Reset to background
                    }
                } else {
                    lines.push_back(std::move(componentPixels)); // Add valid component to lines
                    label++; // Increment label for the next valid connected component
                }
            }
        }
    }

    return label - 1; // Return the number of valid labels
}



// Function to draw the points on the image
void drawPoints(cv::Mat& image, const std::vector<cv::Point2f>& points, const cv::Scalar& color) {
    for (const auto& point : points) {
        cv::circle(image, point, 3, color, -1);  // Draw each point as a small circle
    }
}

/*// Lambda function to sort based on distance to the reference point
auto distanceComparator = [&](const std::vector<cv::Point>& line1, const std::vector<cv::Point>& line2) {
    cv::Point centroid1 = calculateCentroid(line1);
    cv::Point centroid2 = calculateCentroid(line2);
    double distance1 = euclidianDistance(centroid1, referencePoint);
    double distance2 = euclidianDistance(centroid2, referencePoint);
    return distance1 < distance2;  // Sort in ascending order of distance
};*/
auto distanceToBottomComparator = [](const std::vector<cv::Point>& line1, const std::vector<cv::Point>& line2) {
   
    // Get the start and end points of both lines
    cv::Point line1Start = line1.front();
    cv::Point line1End = line1.back();
    cv::Point line2Start = line2.front();
    cv::Point line2End = line2.back();

    // Calculate the vertical distances (y-distance) from the start and end points to the bottom of the image
    int distance1Start = frameHeight - line1Start.y;
    int distance1End = frameHeight - line1End.y;
    int distance2Start = frameHeight - line2Start.y;
    int distance2End = frameHeight - line2End.y;

    // Get the minimum distance for each line (from either end to the bottom)
    int minDistance1 = std::min(distance1Start, distance1End);
    int minDistance2 = std::min(distance2Start, distance2End);

    // Sort based on the minimum distance to the bottom of the image
    return minDistance1 < minDistance2;  // Sort in ascending order of minimum distance to the bottom
};

cv::Mat skeletonizeFrame(const cv::Mat& thresholdedImage){
    cv::Mat skeleton(cv::Mat::zeros(thresholdedImage.size(), CV_8UC1));
    cv::Mat temp, eroded;
    cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));

    while (true) {
        cv::erode(thresholdedImage, eroded, element);
        cv::dilate(eroded, temp, element);
        cv::subtract(thresholdedImage, temp, temp);
        cv::bitwise_or(skeleton, temp, skeleton);
        eroded.copyTo(thresholdedImage);

        if (cv::countNonZero(thresholdedImage) == 0) {
            break;
        }
    }

    /*// Visualize
    cv::Mat outputImage = skeleton.clone();
    // Display the image with lines colored
    cv::imshow("skeleton", outputImage);*/
    
    return skeleton;
}
// Function to find lines in a thinned image
std::vector<std::vector<cv::Point>> findLines(const cv::Mat& thresholdedImage){

    //cv::Mat skeleton = skeletonizeFrame(thresholdedImage);
    // Perform connected component analysis
    cv::Mat labels;
    //int numLabels = cv::connectedComponents(thresholdedImage, labels, 8, CV_32S);
    std::vector<std::vector<cv::Point>> lines;
    int numLabels = customConnectedComponentsWithThreshold(thresholdedImage, labels, 6,lines);

    // Sort the lines based on their distance to the reference point
    //std::sort(lines.begin(), lines.end(), distanceToBottomComparator);
    
    // Return only the first 4 lines (or fewer if there are less than 4)
    if (lines.size() > 4) {
        lines.resize(4);
    }
    
    return lines;
}

// Function to smooth a vector of points using a moving average filter
std::vector<cv::Point> smoothPoints(const std::vector<cv::Point>& points, int windowSize) {
    
    std::vector<cv::Point> smoothedPoints;

    if (windowSize < 1) {
        std::cerr << "Window size must be greater than or equal to 1." << std::endl;
        return points;
    }

    for (int i = 0; i < points.size(); ++i) {
        int sumX = 0, sumY = 0;
        int count = 0;


        // Window range must be within bounds of the points vector
        int start = std::max(0, (int)i - windowSize / 2);
        int end = std::min((int)points.size(), (int)i + windowSize / 2 + 1);

        // Calculate the average of neighboring points
        for (int j = start; j < end; ++j) {
            sumX += points[j].x;
            sumY += points[j].y;
            count++;
        }

        if (count > 0) {
            smoothedPoints.push_back(cv::Point(sumX / count, sumY / count));
        }
    }

    return smoothedPoints;
}

// Function to fit a polynomial to a set of points and return interpolated points
std::vector<cv::Point> fitPolinomial(const std::vector<cv::Point>& line, bool isMiddleLine) {

    std::vector<cv::Point> smoothedLine;

    if(!isMiddleLine){
        // Smooth the points of the current line using the moving average filter
        smoothedLine = smoothPoints(line, windowSize);
    }else{
        smoothedLine = line;
    }

    // Approximate the contours to further smooth them
    cv::approxPolyDP(smoothedLine, smoothedLine, epsilon, false);  // false = open curve  
    
    //extendLineToEdges(smoothedLine);
    return smoothedLine;

}

// Function to evenly space points along the curve based on arc length
std::vector<cv::Point> evenlySpacePoints(const std::vector<cv::Point>& line, int num_points) {
    std::vector<cv::Point> spacedPoints;
    
    // Step 1: Calculate the total arc length of the contour
    std::vector<float> arcLengths(line.size(), 0.0f);
    for (int i = 1; i < line.size(); ++i) {
        arcLengths[i] = arcLengths[i - 1] + static_cast<float>(euclidianDistance(line[i - 1], line[i]));
    }
    float totalArcLength = arcLengths.back();

    // Step 2: Define the target spacing between points
    float spacing = totalArcLength / (num_points - 1);

    // Step 3: Interpolate points based on arc length
    spacedPoints.push_back(line.front());  // Add the first point
    float currentArcLength = spacing;

    for (int i = 1; i < num_points - 1; ++i) {
        // Find where the currentArcLength falls in the original contour
        for (int j = 1; j < line.size(); ++j) {
            if (arcLengths[j] >= currentArcLength) {
                // Linear interpolation between points j-1 and j
                float ratio = (currentArcLength - arcLengths[j - 1]) / (arcLengths[j] - arcLengths[j - 1]);
                float x = line[j - 1].x + ratio * (line[j].x - line[j - 1].x);
                float y = line[j - 1].y + ratio * (line[j].y - line[j - 1].y);
                spacedPoints.push_back(cv::Point(x, y));
                break;
            }
        }
        currentArcLength += spacing;
    }

    spacedPoints.push_back(line.back());  // Add the last point

    return spacedPoints;
}

void extendLineToEdges(std::vector<cv::Point>& middleLine) {
    // Ensure the line has at least two points
    if (middleLine.size() < 2) {
        std::cerr << "Line must have at least two points to extend." << std::endl;
        return;
    }

    // Get the first and second points to calculate the slope at the bottom end
    cv::Point firstPoint = middleLine[0];
    cv::Point secondPoint = middleLine[1];

    // Calculate the slope for extending the line at the bottom end
    float deltaXBottom = secondPoint.x - firstPoint.x;
    float deltaYBottom = secondPoint.y - firstPoint.y;

    // Handle vertical lines (deltaX == 0)
    if (deltaXBottom == 0) {
        deltaXBottom = 1e-6f; // Avoid division by zero
    }
    float slopeBottom = deltaYBottom / deltaXBottom;

    // Extend the line to the bottom of the frame (y = frameHeight)
    float xBottom = firstPoint.x + ((frameHeight - firstPoint.y) / slopeBottom);
     if (xBottom < 0 - frameWidth) xBottom = -frameWidth; // Manually constrain to 2 times the left edge
    if (xBottom >= 2 * frameWidth) xBottom = 2*frameWidth; // Manually constrain to 2 times the right edge
    cv::Point extendedBottom(static_cast<int>(xBottom), frameHeight);

    // Get the second-to-last and last points to calculate the slope at the top end
    cv::Point lastPoint = middleLine.back();
    cv::Point secondLastPoint = middleLine[middleLine.size() - 2];

    float deltaXTop = lastPoint.x - secondLastPoint.x;
    float deltaYTop = lastPoint.y - secondLastPoint.y;

    // Handle vertical lines (deltaX == 0)
    if (deltaXTop == 0) {
        deltaXTop = 1e-6f; // Avoid division by zero
    }
    float slopeTop = deltaYTop / deltaXTop;

    // Extend the line to the top of the frame (y = 0)
    float xTop = lastPoint.x - (lastPoint.y / slopeTop);
    if (xTop < 0 - frameWidth) xTop = -frameWidth; // Manually constrain to 2 times the left edge
    if (xTop >= 2 * frameWidth) xTop = 2*frameWidth; // Manually constrain to 2 times the right edge
    cv::Point extendedTop(static_cast<int>(xTop), 0);

    // Add the extended points to the line
    middleLine.insert(middleLine.begin(), extendedBottom); // Add to the beginning
    middleLine.push_back(extendedTop);                    // Add to the end
}

// Function to find the middle line between two lines
std::vector<cv::Point> findMiddle(std::vector<cv::Point>& leftFitted, std::vector<cv::Point>& rightFitted){
    
    std::vector<cv::Point> allMidpoints;

    for (int i = 1; i < leftFitted.size() && i < rightFitted.size(); i++) {
        // Calculate the midpoint between the corresponding points on the fitted curves
        cv::Point2f midpoint = calculateMidpoint(leftFitted[i], rightFitted[i]);
        cv::Point2f prevMidpoint = calculateMidpoint(leftFitted[i - 1], rightFitted[i - 1]);

        allMidpoints.push_back(prevMidpoint);        
    }
    
    // Include the last point
    if (!leftFitted.empty() && !rightFitted.empty()) {
        cv::Point2f lastMidpoint = calculateMidpoint(leftFitted.back(), rightFitted.back());
        allMidpoints.push_back(lastMidpoint);
    }
    
    //allMidpoints = fitPolinomial(allMidpoints,true);
    extendLineToEdges(allMidpoints);
    return allMidpoints;
}

bool are2PointsHorizontal(const cv::Point2f& p1, const cv::Point2f& p2, double slopeThreshold = 0.3) {
    float deltaX = p2.x - p1.x;
    float deltaY = p2.y - p1.y;

    // Avoid division by zero (in case of vertical lines)
    if (deltaX == 0) {
        return false;  // It's a vertical line
    }

    // Calculate the slope
    float slope = deltaY / deltaX;

    // Check if the slope is close to zero (indicating a horizontal line)
    return std::abs(slope) <= slopeThreshold;
}

// Function to remove horizontal sections if a 90-degree turn is detected
std::vector<cv::Point> removeHorizontalIf90Turn(const cv::Mat& frame,const std::vector<cv::Point>& line) {
    bool has90DegreeTurn = false;
    std::vector<cv::Point> result;

    // Check for 90-degree turns along the line
    for (int i = 1; i < line.size() - 1; ++i) {
        double angle = calculateAngle(line[i - 1], line[i], line[i + 1]);
        if (angle > 65 && angle < 115) {  // Close to 90 degrees
            has90DegreeTurn = true;
            break;
        }
    }

    // If there is a 90-degree turn, remove the horizontal parts
    if (has90DegreeTurn) {
        for (int i = 1; i < line.size(); ++i) {
            if (!are2PointsHorizontal(line[i - 1], line[i],slopeThreshold)) {
                result.push_back(line[i - 1]);  // Keep only non-horizontal parts
            }
        }
    } else {
        result = line;  // No turn, keep the entire line
    }

    return result;
}

#endif 