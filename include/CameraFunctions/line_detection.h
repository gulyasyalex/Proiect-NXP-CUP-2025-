#ifndef LINE_DETECTION_H
#define LINE_DETECTION_H

#include <opencv2/opencv.hpp>
#include <cmath>
#include "config.h"
#include "thinning.h"
#include "camera_setup.h"


/*
    Rises to the right (top-right):     m<0 (negative slope)
    Falls to the right (bottom-right):  m>0 (positive slope)
    Rises to the left (top-left):       m>0 (positive slope)
    Falls to the left (bottom-left):    m<0 (negative slope)
*/

cv::Point undefinedPoint = cv::Point(1000,0);
cv::Point firstPointLeftLine = undefinedPoint;
cv::Point firstPointRightLine = undefinedPoint;
cv::Point firstPointSingleLine = undefinedPoint;

double lineLength(const cv::Vec4i& line);
double euclidianDistance(cv::Point p1, cv::Point p2);
cv::Point calculateMidpoint(const cv::Point& p1, const cv::Point& p2);
cv::Point calculateCentroid(const std::vector<cv::Point>& line);

std::vector<cv::Point> generateNeighborhood(int radius);
int customConnectedComponentsWithThreshold(const cv::Mat& binaryImage, cv::Mat& labelImage, int radius, std::vector<std::vector<cv::Point>>& lines);

void extendLineToEdges(std::vector<cv::Point>& line, int providedFrameWidth, int providedFrameHeight);
std::vector<std::vector<cv::Point>> findLines(const cv::Mat& thresholdedImage);
std::vector<cv::Point> smoothPoints(const std::vector<cv::Point>& points, int windowSize);
std::vector<std::vector<cv::Point>> fitPolinomial(const cv::Mat& frame, std::vector<std::vector<cv::Point>> &lines);
std::vector<cv::Point> evenlySpacePoints(const std::vector<cv::Point>& line, int num_points);
bool are2PointsHorizontal(const cv::Point2f& p1, const cv::Point2f& p2, double slopeThreshold);
std::vector<cv::Point> removeHorizontalIf90Turn(const std::vector<cv::Point>& line);
void getLeftRightLines(const std::vector<std::vector<cv::Point>>& lines, std::vector<cv::Point>& leftLine, std::vector<cv::Point>& rightLine);
std::vector<cv::Point> findMiddle(std::vector<cv::Point>& leftFitted, std::vector<cv::Point>& rightFitted);
cv::Point interpolateClosestPoints(const std::vector<cv::Point>& points, int targetY);

void drawPoints2f(cv::Mat& frame, const std::vector<cv::Point2f>& points, const cv::Scalar& color);
void drawPoints(cv::Mat& frame, const std::vector<cv::Point>& points, const cv::Scalar& color);
void drawLine(cv::Mat& frame, const std::vector<cv::Point>& points, const cv::Scalar& color);
void drawLines(cv::Mat& frame, const std::vector<std::vector<cv::Point>> lines, const cv::Scalar& color);

// Function to calculate the length of a line
double lineLength(const cv::Vec4i& line) {
    int x1 = line[0], y1 = line[1], x2 = line[2], y2 = line[3];
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}
// Function to calculate the Euclidean distance between two OpenCv points
double euclidianDistance(cv::Point p1, cv::Point p2) {
   return std::hypot(p1.x - p2.x, p1.y - p2.y);
}
// Function to calculate the Euclidean distance between two points in coordinate values
double euclideanDistanceCoord(int x1, int y1, int x2, int y2) {
    return std::hypot(x1 - x2, y1 - y2);
}
// Function to calculate the angle between three points used in removeHorizontalIf90Turn
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
// Function that generates a radius of values for customConnectedComponentsWithThreshold
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
// Returns lines found in image based on some requirements
int customConnectedComponentsWithThreshold(const cv::Mat& binaryImage, cv::Mat& labelImage, int radius, std::vector<std::vector<cv::Point>>& lines) {
    
    // This algorithm start from bottom left

    CV_Assert(binaryImage.type() == CV_8UC1);
    labelImage = cv::Mat::zeros(binaryImage.size(), CV_32S); // Initialize label matrix
    lines.clear(); // Clear any existing lines

    int label = 1; // Start labeling from 1
    const int rowThreshold = static_cast<int>(binaryImage.rows * rowThresholdCutOff); // Top 40% cutoff to mitigate Far View error
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

// Function used in findLines to sort line based on distance from bottom
auto distanceToBottomComparator = [](const std::vector<cv::Point>& line1, const std::vector<cv::Point>& line2) {
   
    // Get the start and end points of both lines
    cv::Point line1Start = line1.front();
    cv::Point line1End = line1.back();
    cv::Point line2Start = line2.front();
    cv::Point line2End = line2.back();

    // Calculate the vertical distances (y-distance) from the start and end points to the bottom of the frame
    int distance1Start = frameHeight - line1Start.y;
    int distance1End = frameHeight - line1End.y;
    int distance2Start = frameHeight - line2Start.y;
    int distance2End = frameHeight - line2End.y;

    // Get the minimum distance for each line (from either end to the bottom)
    int minDistance1 = std::min(distance1Start, distance1End);
    int minDistance2 = std::min(distance2Start, distance2End);

    // Sort based on the minimum distance to the bottom of the frame
    return minDistance1 < minDistance2;  // Sort in ascending order of minimum distance to the bottom
};

// Function to find lines in a skeletonized frame
std::vector<std::vector<cv::Point>> findLines(const cv::Mat& thresholdedImage){

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
// Function to smooth a vector of points using a moving average filter used in fitPolinomial
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
void extendLineToEdges(std::vector<cv::Point>& line, int providedFrameWidth, int providedFrameHeight) {
    // Ensure the line has at least two points
    if (line.size() < 2) {
        std::cerr << "Line must have at least two points to extend." << std::endl;
        return;
    }
    cv::Point firstBottomPoint;
    cv::Point secondBottomPoint;
    cv::Point firstTopPoint;
    cv::Point secondTopPoint;
    

    // Get the second-to-last and last points to calculate the slope at the top end
    firstTopPoint = line.back();
    secondTopPoint = line[line.size() - 2];
    // Get the first and second points to calculate the slope at the bottom end
    firstBottomPoint = line[0];
    secondBottomPoint = line[1];
    

    // Calculate the slope for extending the line at the bottom end
    double deltaXBottom = secondBottomPoint.x - firstBottomPoint.x;
    double deltaYBottom = secondBottomPoint.y - firstBottomPoint.y;
    
    // Handle vertical lines (deltaX == 0)
    if (deltaXBottom == 0) {
        deltaXBottom = 1e-6f; // Avoid division by zero
    }
    double slopeBottom = deltaYBottom / deltaXBottom;

    // Extend the line to the bottom of the frame (y = providedFrameHeight)
    double xBottom = firstBottomPoint.x + ((providedFrameHeight - firstBottomPoint.y) / slopeBottom);
    if (xBottom < 0 - providedFrameWidth) xBottom = -providedFrameWidth; // Manually constrain to 2 times the left edge in case line is too long
    if (xBottom >= 2 * providedFrameWidth) xBottom = 2*providedFrameWidth; // Manually constrain to 2 times the right edge in case line is too long
    cv::Point extendedBottom(static_cast<int>(xBottom), providedFrameHeight);

    // Add the extended point to the line
    line.insert(line.begin(), extendedBottom); // Add to the beginning
    

    double deltaXTop = firstTopPoint.x - secondTopPoint.x;
    double deltaYTop = firstTopPoint.y - secondTopPoint.y;
    
    // Handle vertical lines (deltaX == 0)
    if (deltaXTop == 0) {
        deltaXTop = 1e-6f; // Avoid division by zero
    }
    double slopeTop = deltaYTop / deltaXTop;
    
    if(slopeTop > 0.15 || slopeTop  < -0.15){
        
        // Extend the line to the top of the frame (y = 0)
        double xTop = firstTopPoint.x - (firstTopPoint.y / slopeTop);
        if (xTop < 0 - providedFrameWidth) xTop = -providedFrameWidth; // Manually constrain to 2 times the left edge
        if (xTop >= 2 * providedFrameWidth) xTop = 2*providedFrameWidth; // Manually constrain to 2 times the right edge
        cv::Point extendedTop(static_cast<int>(xTop), 0);
        // Add the extended points to the line
        line.push_back(extendedTop);                    // Add to the end
    }
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
    return smoothedLine;
}
// Function to evenly space points along the curve based on arc length
std::vector<cv::Point> evenlySpacePoints(const std::vector<cv::Point>& line, int num_points) {
    std::vector<cv::Point> spacedPoints;
    
    // Step 1: Calculate the total arc length of the contour
    std::vector<double> arcLengths(line.size(), 0.0f);
    for (int i = 1; i < line.size(); ++i) {
        arcLengths[i] = arcLengths[i - 1] + static_cast<double>(euclidianDistance(line[i - 1], line[i]));
    }
    double totalArcLength = arcLengths.back();

    // Step 2: Define the target spacing between points
    double spacing = totalArcLength / (num_points - 1);

    // Step 3: Interpolate points based on arc length
    spacedPoints.push_back(line.front());  // Add the first point
    double currentArcLength = spacing;

    for (int i = 1; i < num_points - 1; ++i) {
        // Find where the currentArcLength falls in the original contour
        for (int j = 1; j < line.size(); ++j) {
            if (arcLengths[j] >= currentArcLength) {
                // Linear interpolation between points j-1 and j
                double ratio = (currentArcLength - arcLengths[j - 1]) / (arcLengths[j] - arcLengths[j - 1]);
                double x = line[j - 1].x + ratio * (line[j].x - line[j - 1].x);
                double y = line[j - 1].y + ratio * (line[j].y - line[j - 1].y);
                spacedPoints.push_back(cv::Point(x, y));
                break;
            }
        }
        currentArcLength += spacing;
    }

    spacedPoints.push_back(line.back());  // Add the last point

    return spacedPoints;
}

bool are2PointsHorizontal(const cv::Point2f& p1, const cv::Point2f& p2, double slopeThreshold = 1) {
    
    double deltaX = p2.x - p1.x;
    double deltaY = p2.y - p1.y;

    // Avoid division by zero (in case of vertical lines)
    if (deltaX == 0) {
        return false;  // It's a vertical line
    }

    // Calculate the slope
    double slope = deltaY / deltaX;
    // Check if the slope is close to zero (indicating a horizontal line)
    return std::abs(slope) <= slopeThreshold;
}
// Function to remove horizontal sections if a 90-degree turn is detected
std::vector<cv::Point> removeHorizontalIf90Turn(const std::vector<cv::Point>& line) {
    bool has90DegreeTurn = false;
    std::vector<cv::Point> result;

    // Check for 90-degree turns along the line
    for (int i = 1; i < line.size() - 1; ++i) {
        double angle = calculateAngle(line[i - 1], line[i], line[i + 1]);
        if (angle > removeAngleMin && angle < removeAngleMax) {  // Close to 90 degrees
            has90DegreeTurn = true;
            std::cout << "Angle(has90DegreeTurn): " << angle << std::endl;
            break;
        }
    }

    // If there is a 90-degree turn, remove the horizontal parts
    if (has90DegreeTurn) {
        
        for (int i = 1; i < line.size(); ++i) {
            if (!are2PointsHorizontal(line[i - 1], line[i], slopeThreshold)) {
                // Add the starting point of the non-horizontal segment
                if (result.empty() || result.back() != line[i - 1]) {
                    result.push_back(line[i - 1]);
                }

                // Add the endpoint of the non-horizontal segment
                if (result.empty() || result.back() != line[i]) {
                    result.push_back(line[i]);
                }
            }
        }

        std::cout << "Line has 90DegreeTurn" << std::endl;
    } else {
        result = line;  // No turn, keep the entire line
        std::cout << "Line is Normal" << std::endl;
    }

    return result;
}
// Function that handles 2 Lines, 1 Line and No line cases
void getLeftRightLines(const std::vector<std::vector<cv::Point>>& lines, std::vector<cv::Point>& leftFitted, std::vector<cv::Point>& rightFitted){

    cv::Point firstPointLineA = undefinedPoint;
    cv::Point firstPointLineB = undefinedPoint;
    cv::Point shiftedPoint;
    double deltaX;
    double deltaY;
    double slope;

    /*
        If we have 2 line it looks to see which one's X value is smaller
        because the frame start from 0,0 and it get's extended to the right
        then the smaller value of X is the left line

        firstPointLineA at first is choosed randomly but after it is set to left line first point
    */
    if(lines.size() >= 2)
    {
        firstPointLineA = lines[0][0];
        firstPointLineB = lines[1][0];

        if( firstPointLineA.x < firstPointLineB.x )
        {
            leftFitted = lines[0];
            firstPointLeftLine = firstPointLineA;
            rightFitted = lines[1];            
            firstPointRightLine = firstPointLineB;
        }
        else
        {
            leftFitted = lines[1];
            firstPointLeftLine = firstPointLineB;
            rightFitted = lines[0];
            firstPointRightLine = firstPointLineA;
        }
        
    }
    /*
        If we have one line and history then decide to which starting point is closer:
            - If it is to the left then duplicate a line to it's left and lower
            - If it is to the right then duplicat a line to it's right and lower

        If we have one line and no history then decide based on slope (corner case)
    */
    else if (1 == lines.size())
    {
        
        firstPointSingleLine = lines[0][0];
        
        if (euclidianDistance(firstPointSingleLine,firstPointLeftLine) <  euclidianDistance(firstPointSingleLine,firstPointRightLine))
        {
            leftFitted = lines[0];
            rightFitted.clear();
            for (const auto& point : leftFitted) {
                shiftedPoint = cv::Point(point.x + trackLaneWidthInPixel, point.y);
                rightFitted.push_back(shiftedPoint);
            }
        }
        else if(euclidianDistance(firstPointSingleLine,firstPointLeftLine) >  euclidianDistance(firstPointSingleLine,firstPointRightLine))
        {
            rightFitted = lines[0];
            leftFitted.clear();
            for (const auto& point : rightFitted) {
                shiftedPoint = cv::Point(point.x - trackLaneWidthInPixel, point.y);
                leftFitted.push_back(shiftedPoint);
            }
        }
        else if( firstPointLeftLine == undefinedPoint || firstPointRightLine == undefinedPoint){
            // Compare to based on slope of first and last points
            // If it has an orientation to the left it is left otherwise rightv
            cv::Point pointBack(lines[0].back().x,lines[0].back().y);
            cv::Point pointFront(lines[0][0].x,lines[0][0].y);

            perspectiveChangePoint(pointBack, MatrixInverseBirdsEyeView);
            perspectiveChangePoint(pointFront, MatrixInverseBirdsEyeView);

            double deltaX = pointBack.x - pointFront.x;
            double deltaY = pointBack.y - pointFront.y;
            // Handle vertical lines (deltaX == 0)
            if (deltaX == 0) {
                deltaX = 1e-6f; // Avoid division by zero
            }
            double slope = deltaY / deltaX;
        
            // If slope rises to the right (top-right) then we have left line
            if(slope < 0)
            {
                leftFitted = lines[0];
                rightFitted.clear();
                for (const auto& point : leftFitted) {
                    shiftedPoint = cv::Point(point.x + trackLaneWidthInPixel, point.y);
                    rightFitted.push_back(shiftedPoint);
                }
            } 
            else
            {
                rightFitted = lines[0];
                leftFitted.clear();
                for (const auto& point : rightFitted) {
                    shiftedPoint = cv::Point(point.x - trackLaneWidthInPixel, point.y);
                    leftFitted.push_back(shiftedPoint);
                }
            }            
        }
    }else{
        firstPointLeftLine = undefinedPoint;
        firstPointRightLine = undefinedPoint;
        // TBD NO LINES, STOP CAR
    }
}

// Function to find the middle line between two lines
std::vector<cv::Point> findMiddle(std::vector<cv::Point>& leftFitted, std::vector<cv::Point>& rightFitted,
 int providedFrameWidth, int providedFrameHeight){
    
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
    
    extendLineToEdges(allMidpoints, providedFrameWidth, providedFrameHeight);
    return allMidpoints;
}

cv::Point interpolateClosestPoints(const std::vector<cv::Point>& points, int targetY) {
    if (points.size() < 2) {
        throw std::invalid_argument("Not enough points to interpolate");
    }

    // Initialize variables to track the closest points
    cv::Point lower, upper;
    bool lowerFound = false, upperFound = false;

    for (const auto& point : points) {
        if (point.y <= targetY) {
            if (!lowerFound || point.y > lower.y) {
                lower = point;
                lowerFound = true;
            }
        }
        if (point.y >= targetY) {
            if (!upperFound || point.y < upper.y) {
                upper = point;
                upperFound = true;
            }
        }
    }

    // Ensure we found bounding points
    if (!lowerFound || !upperFound) {
        throw std::invalid_argument("Target Y is out of bounds of the given points");
    }

    // Linear interpolation to find x
    int interpolatedX = lower.x + (targetY - lower.y) * (upper.x - lower.x) / (upper.y - lower.y);
    return cv::Point(interpolatedX, targetY);
}
// Function to draw the points on the frame
void drawPoints2f(cv::Mat& frame, const std::vector<cv::Point2f>& points, const cv::Scalar& color) {
    for (const auto& point : points) {
        cv::circle(frame, point, 5, color, -1);
    }
}void drawPoints(cv::Mat& frame, const std::vector<cv::Point>& points, const cv::Scalar& color) {
    for (const auto& point : points) {
        cv::circle(frame, point, 5, color, -1);
    }
}
// Function to draw the line on the frame
void drawLine(cv::Mat& frame, const std::vector<cv::Point>& line, const cv::Scalar& color) {
    for (int i = 1; i < line.size(); i++) {
        cv::line(frame, line[i-1], line[i], color, 2);  // Green
    }
}
// Function to draw the line on the frame
void drawLines(cv::Mat& frame, const std::vector<std::vector<cv::Point>> lines, const cv::Scalar& color) {
    for (int i = 0; i < lines.size(); ++i) {
        for (int j = 0; j < lines[i].size() - 1; ++j) {
            cv::line(frame, lines[i][j], lines[i][j + 1], color, 2);
        }
    }
}
#endif 