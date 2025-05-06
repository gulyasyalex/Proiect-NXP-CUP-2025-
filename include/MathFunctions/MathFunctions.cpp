#include "MathFunctions/MathFunctions.hpp"

// Function to calculate the length of a line
double lineLength(const cv::Vec4i& line) {
    int x1 = line[0], y1 = line[1], x2 = line[2], y2 = line[3];
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}
// Function to calculate the Euclidean distance between two OpenCv points
double euclideanDistance(cv::Point2f p1, cv::Point2f p2) {
   return std::hypot(p1.x - p2.x, p1.y - p2.y);
}
// Function to calculate the Euclidean distance between two points in coordinate values
double euclideanDistanceCoord(int x1, int y1, int x2, int y2) {
    return std::hypot(x1 - x2, y1 - y2);
}

// Function to calculate the signed angle between three points 
double calculateSignedAngleThreePoints(cv::Point2f p1, cv::Point2f p2, cv::Point2f p3)
{
    cv::Point2f vec1 = p1 - p2; // Vector de la p2 la p1
    cv::Point2f vec2 = p3 - p2; // Vector de la p2 la p3
    double dotProduct = vec1.x * vec2.x + vec1.y * vec2.y;
    double crossProduct = vec1.x * vec2.y - vec1.y * vec2.x;
    //double angle = std::atan2(crossProduct, dotProduct) * 180.0 / CV_PI;
    double angle = std::atan2(crossProduct, dotProduct);
    return angle;
}

// Function to calculate the midpoint between two points
cv::Point2f calculateMidpoint(const cv::Point2f& p1, const cv::Point2f& p2) 
{
    return cv::Point2f((p1.x + p2.x) / 2, (p1.y + p2.y) / 2);
}
// Function to calculate the centroid of a line (average of the points)
cv::Point2f calculateCentroid(const std::vector<cv::Point2f>& line) 
{
    cv::Point2f centroid(0, 0);
    for (const auto& point : line) {
        centroid.x += point.x;
        centroid.y += point.y;
    }
    centroid.x /= line.size();
    centroid.y /= line.size();
    return centroid;
}
// Function that generates a radius of values for customConnectedComponentsWithThreshold
std::vector<cv::Point2f> generateNeighborhood(int radius) 
{
    std::vector<cv::Point2f> neighbors;
    for (int dy = -radius; dy <= radius; ++dy) {
        for (int dx = -radius; dx <= radius; ++dx) {
            if (std::sqrt(dx * dx + dy * dy) <= radius) { // Ensure point lies within the circle
                neighbors.emplace_back(dx, dy);
            }
        }
    }
    return neighbors;
}

// Function to find the closest segment as a vector of two points
std::vector<cv::Point2f> closestSegmentOnCurve(const std::vector<cv::Point2f>& curve, const cv::Point2f& point) 
{
    double minDistance = std::numeric_limits<double>::max();
    std::vector<cv::Point2f> bestSegment(2); // Stores the closest segment

    for (size_t i = 0; i < curve.size() - 1; ++i) {
        cv::Point2f p1 = curve[i];
        cv::Point2f p2 = curve[i + 1];

        // Compute the projection of the point onto the segment
        cv::Point2f lineVec = p2 - p1;
        cv::Point2f pointVec = point - p1;
        float t = lineVec.dot(pointVec) / lineVec.dot(lineVec);

        cv::Point2f closestPoint;
        if (t < 0.0f) {
            closestPoint = p1;
        } else if (t > 1.0f) {
            closestPoint = p2;
        } else {
            closestPoint = p1 + t * lineVec;
        }

        double dist = euclideanDistance(point, closestPoint);
        if (dist < minDistance) {
            minDistance = dist;
            bestSegment[0] = p1;
            bestSegment[1] = p2;
        }
    }
    return bestSegment;
}


// Function to compute the angle between two vectors
double computeAngleBetweenVectors(const cv::Point2f& v1, const cv::Point2f& v2) 
{
    double dot = v1.x * v2.x + v1.y * v2.y;
    double mag1 = std::sqrt(v1.x * v1.x + v1.y * v1.y);
    double mag2 = std::sqrt(v2.x * v2.x + v2.y * v2.y);

    if (mag1 < 1e-5 || mag2 < 1e-5) {
        return 0.0; // Vectors too small to define a reliable angle
    }

    double cosAngle = dot / (mag1 * mag2);
    cosAngle = std::max(-1.0, std::min(1.0, cosAngle)); // Prevents NaN from acos due to floating-point precision

    return std::acos(cosAngle) * 180.0 / CV_PI;
}


double roundToTwoDecimals(double value) {
    return std::round(value * 100.0) / 100.0;
}

std::string to_string_with_precision(double value, int precision = 2) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(precision) << value;
    return stream.str();
}

int doubleCmp(double num1, double num2) {
    if (std::fabs(num1 - num2) < DBL_EPSILON) {
        return 0;   // approximately equal
    } else if (num1 > num2) {
        return 1;
    }
    return -1;
}

double mapDouble(double x, double in_min, double in_max, double out_min, double out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Calculates perpendicular distance from point p to line segment (start, end)
float perpendicularDistance(const cv::Point2f& p, const cv::Point2f& start, const cv::Point2f& end) {
    float dx = end.x - start.x;
    float dy = end.y - start.y;

    if (dx == 0 && dy == 0)
        return cv::norm(p - start);

    float numerator = std::abs(dy * p.x - dx * p.y + end.x * start.y - end.y * start.x);
    float denominator = std::sqrt(dx * dx + dy * dy);
    return numerator / denominator;
}

/* Ramer–Douglas–Peucker algorithm */
void rdpSimplify(const std::vector<cv::Point2f>& input, double epsilon, std::vector<cv::Point2f>& output) {
    if (input.size() < 2) {
        output = input;
        return;
    }

    // Find the point with the maximum perpendicular distance
    double maxDist = 0.0;
    size_t index = 0;

    const cv::Point2f& start = input.front();
    const cv::Point2f& end = input.back();

    for (size_t i = 1; i < input.size() - 1; ++i) {
        // Calculate perpendicular distance from input[i] to line (start - end)
        double num = std::abs((end.y - start.y) * input[i].x - (end.x - start.x) * input[i].y + end.x * start.y - end.y * start.x);
        double den = euclideanDistance(start, end);
        double dist = den > 0 ? num / den : 0;

        if (dist > maxDist) {
            index = i;
            maxDist = dist;
        }
    }

    // If max distance is greater than epsilon, recursively simplify
    if (maxDist > epsilon) {
        std::vector<cv::Point2f> left(input.begin(), input.begin() + index + 1);
        std::vector<cv::Point2f> right(input.begin() + index, input.end());

        std::vector<cv::Point2f> simplifiedLeft, simplifiedRight;
        rdpSimplify(left, epsilon, simplifiedLeft);
        rdpSimplify(right, epsilon, simplifiedRight);

        // Merge results, remove duplicate point at boundary
        output = simplifiedLeft;
        output.pop_back();
        output.insert(output.end(), simplifiedRight.begin(), simplifiedRight.end());
    } else {
        output = { start, end };
    }
}

