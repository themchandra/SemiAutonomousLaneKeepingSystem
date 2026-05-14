#include "LaneDetection.h"

#include <cstdlib>
#include <ctime>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <filesystem>

cv::Mat LaneDetection::s_frame;
int LaneDetection::s_frameCenter;
int LaneDetection::s_maxLineHeight;
int LaneDetection::s_laneCenter = 0;
std::array<std::array<cv::Point, 4>, LaneDetection::s_hystheresisCount> LaneDetection::s_hystheresisArray = {{}};
unsigned short LaneDetection::s_hystheresisArrayCounter = 0;
bool LaneDetection::s_hystheresisArrayFilled = false;
cv::Mat LaneDetection::s_mask;
std::vector<cv::Vec4i> LaneDetection::s_lines;
std::vector<cv::Point> LaneDetection::s_rightLinePoints;
std::vector<cv::Point> LaneDetection::s_leftLinePoints;
std::array<cv::Point, 4> LaneDetection::s_boundaries = {};
bool LaneDetection::s_previewEnabled = true;

int LaneDetection::s_steeringError = 0;

void LaneDetection::computeLaneCenter()
{
    s_laneCenter = (s_boundaries[0].x + s_boundaries[3].x) / 2;
}

void LaneDetection::computeSteeringError()
{
    s_steeringError = s_laneCenter - s_frameCenter;
}

int LaneDetection::getSteeringError()
{
    return s_steeringError;
}

static void ensureDebugDirectories()
{
    namespace fs = std::filesystem;
    try
    {
        fs::create_directories("debug/input");
        fs::create_directories("debug/roi");
        fs::create_directories("debug/mask");
        fs::create_directories("debug/morphology");
        fs::create_directories("debug/hough");
        fs::create_directories("debug/final");
    }
    catch (const std::exception &e)
    {
        std::cerr << "Failed to create debug directories: " << e.what() << "\n";
    }
}

bool LaneDetection::hasDisplayServer()
{
    const char *display = std::getenv("DISPLAY");
    const char *waylandDisplay = std::getenv("WAYLAND_DISPLAY");

    return (display != nullptr && display[0] != '\0') || (waylandDisplay != nullptr && waylandDisplay[0] != '\0');
}

void LaneDetection::createMask(const cv::Size &frameSize, double frameFormat)
{
    (void)frameFormat;
    s_mask = cv::Mat::zeros(frameSize, CV_8UC1);

    cv::rectangle(
        s_mask,
        cv::Point(0, static_cast<int>(frameSize.height * 0.45)),
        cv::Point(frameSize.width, frameSize.height),
        cv::Scalar(255),
        cv::FILLED);
}

inline void LaneDetection::applyMask()
{

    if (s_frame.channels() == 1)
    {
        cv::bitwise_and(s_frame, s_mask, s_frame);
        return;
    }

    std::vector<cv::Mat> channels;
    cv::split(s_frame, channels);

    for (auto &channel : channels)
    {
        cv::bitwise_and(channel, s_mask, channel);
    }

    cv::merge(channels, s_frame);
}

inline void LaneDetection::changeContrast()
{
    // cv::convertScaleAbs(s_frame, s_frame, 1.3, 0);
}

inline void LaneDetection::blur()
{
    cv::GaussianBlur(s_frame, s_frame, cv::Size(7, 7), 0, 0); // 7x7px trial & error
}

inline void LaneDetection::edgeDetection()
{
    // Build a robust mask for white lane markings by combining
    // - HSV white detection (low saturation, high value)
    // - intensity threshold (bright pixels in grayscale)
    // Then clean and optionally edge-enhance for Hough.

    cv::Mat hsv;
    cv::cvtColor(s_frame, hsv, cv::COLOR_BGR2HSV);
    cv::imwrite("debug/input/01_hsv.png", hsv);

    cv::Mat whiteHSV;
    // broaden saturation and value ranges to be more robust to lighting
    cv::inRange(hsv, cv::Scalar(0, 0, 110), cv::Scalar(180, 140, 255), whiteHSV);
    cv::imwrite("debug/mask/02_whiteHSV.png", whiteHSV);
    std::cout << "    whiteHSV non-zero pixels: " << cv::countNonZero(whiteHSV) << "\n";

    cv::Mat gray;
    cv::cvtColor(s_frame, gray, cv::COLOR_BGR2GRAY);
    cv::imwrite("debug/input/03_gray.png", gray);

    cv::Mat bright;
    // simple global threshold; may be adjusted (try 180-220)
    cv::threshold(gray, bright, 130, 255, cv::THRESH_BINARY);
    cv::imwrite("debug/mask/04_bright.png", bright);
    std::cout << "    bright non-zero pixels: " << cv::countNonZero(bright) << "\n";

    // Combine both masks
    cv::Mat combined;
    cv::bitwise_or(whiteHSV, bright, combined);
    cv::imwrite("debug/mask/05_combined.png", combined);
    std::cout << "    combined non-zero pixels: " << cv::countNonZero(combined) << "\n";

    // Clean mask: open then close to remove small noise and bridge gaps
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::Mat cleaned;
    cv::morphologyEx(combined, cleaned, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(cleaned, cleaned, cv::MORPH_CLOSE, kernel);
    cv::imwrite("debug/morphology/06_cleaned.png", cleaned);
    std::cout << "    cleaned non-zero pixels: " << cv::countNonZero(cleaned) << "\n";

    // Optional: use Canny edges to give cleaner inputs to HoughLinesP
    cv::Mat edges;
    cv::Canny(cleaned, edges, 50, 150);
    cv::imwrite("debug/hough/07_edges.png", edges);
    std::cout << "    edges non-zero pixels: " << cv::countNonZero(edges) << "\n";

    // Keep the final output in s_frame only after every intermediate stage is saved.
    if (cv::countNonZero(edges) > 50)
    {
        s_frame = edges;
    }
    else
    {
        s_frame = cleaned;
    }
}

inline void LaneDetection::houghLines()
{
    s_lines.clear();

    // Tune parameters: increase minLineLength and reduce maxLineGap
    // for longer continuous markings on tracks.
    cv::HoughLinesP(s_frame, s_lines, 1, CV_PI / 180, 18, 40, 20);
}

void LaneDetection::classifyLines()
{
    s_rightLinePoints.clear();
    s_leftLinePoints.clear();

    const float minSlope = 0.3f;
    const float maxSlope = 1.5f;

    for (const auto &line : s_lines)
    {

        // slope = (y1 - y0) / (x1 - x0)
        float slope = static_cast<float>(line[3] - line[1]);
        slope /= (static_cast<float>(line[2] - line[0]) + 0.00001f);

        // filter too horizontal slopes
        float absSlope = std::fabs(slope);
        if (absSlope < minSlope || absSlope > maxSlope)
            continue;

        if (slope > 0 && line[2] > s_frameCenter && line[0] > s_frameCenter)
        {
            s_rightLinePoints.push_back(cv::Point(line[0], line[1]));
            s_rightLinePoints.push_back(cv::Point(line[2], line[3]));
        }
        else if (slope < 0 && line[2] < s_frameCenter && line[0] < s_frameCenter)
        {
            s_leftLinePoints.push_back(cv::Point(line[0], line[1]));
            s_leftLinePoints.push_back(cv::Point(line[2], line[3]));
        }
    }
}

void LaneDetection::leastSquaresRegression()
{

    std::array<float, 4> xPositions = {0.f, 0.f, 0.f, 0.f};
    float left_m = 0.0f;
    float right_m = 0.0f;
    // Use proportional image coordinates for lower/upper interpolation targets
    // Avoid extrapolating all the way to the bottom which amplifies slope noise.
    int lowerY = static_cast<int>(s_frame.rows * 0.60f); // suggested: 60% down
    int upperY = static_cast<int>(s_frame.rows * 0.50f); // suggested upper target

    // fit left lane
    if (!s_leftLinePoints.empty())
    {
        cv::Vec4d left_line;

        cv::fitLine(s_leftLinePoints, left_line, cv::DIST_L2, 0, 0.01, 0.01);
        left_m = left_line[1] / left_line[0];
        cv::Point left_b = cv::Point(left_line[2], left_line[3]);

        xPositions[0] = (static_cast<float>(lowerY - left_b.y) / left_m) + left_b.x; // lower
        xPositions[1] = (static_cast<float>(upperY - left_b.y) / left_m) + left_b.x; // upper
    }

    // fit right lane
    if (!s_rightLinePoints.empty())
    {
        cv::Vec4d right_line;

        cv::fitLine(s_rightLinePoints, right_line, cv::DIST_L2, 0, 0.01, 0.01);
        right_m = right_line[1] / right_line[0];
        cv::Point right_b = cv::Point(right_line[2], right_line[3]); // y = m*x + b

        xPositions[2] = (static_cast<float>(lowerY - right_b.y) / right_m) + right_b.x; // lower
        xPositions[3] = (static_cast<float>(upperY - right_b.y) / right_m) + right_b.x; // upper
    }
    std::cout << "Raw xPositions before clamp: "

              << xPositions[0] << ", "

              << xPositions[1] << ", "

              << xPositions[2] << ", "

              << xPositions[3] << "\n";
    std::cout << "Left m: " << left_m << "\n";

    std::cout << "Right m: " << right_m << "\n";

    // Clamp x positions to image bounds
    auto clampX = [](float x, int width)
    {
        return std::max(0.0f,
                        std::min(x, static_cast<float>(width - 1)));
    };

    for (float &x : xPositions)
    {
        x = clampX(x, s_frame.cols);
    }

    hystheresis(xPositions, lowerY, upperY);
}

inline void LaneDetection::hystheresis(std::array<float, 4> xPositions, int lowerY, int upperY)
{

    s_hystheresisArray[s_hystheresisArrayCounter][0] = cv::Point(xPositions[0], lowerY);
    s_hystheresisArray[s_hystheresisArrayCounter][1] = cv::Point(xPositions[1], upperY);
    s_hystheresisArray[s_hystheresisArrayCounter][2] = cv::Point(xPositions[2], lowerY);
    s_hystheresisArray[s_hystheresisArrayCounter][3] = cv::Point(xPositions[3], upperY);

#ifdef USE_HYSTHERESIS
    if (s_hystheresisArrayFilled)
    {

        std::array<cv::Point, 4> previousRow;
        if (s_hystheresisArrayCounter == 0)
        {
            previousRow = s_hystheresisArray[s_hystheresisCount - 1];
        }
        else
        {
            previousRow = s_hystheresisArray[s_hystheresisArrayCounter - 1];
        }

        const int maxLowerDiff = 0.01f * s_frame.cols;
        const int maxUpperDiff = 0.004f * s_frame.cols;

        std::array<int, 4> maxDiff = {maxLowerDiff, maxUpperDiff, maxLowerDiff, maxUpperDiff};
        std::array<int, 4> avgXPositions = {};

        // unsigned short attempts = 0;

        // average over whole array (excluding 0-values)
        for (unsigned short i = 0; i < 4; i++)
        {
            unsigned short skipped = 0;
            int avg = 0;

            for (unsigned short j = 0; j < s_hystheresisCount; j++)
            {

                if (s_hystheresisArray[j][i].x == 0)
                    skipped++;
                avg += s_hystheresisArray[j][i].x;
            }

            avgXPositions[i] = avg;
            if (s_hystheresisCount == skipped)
            {
                errorHandler();
                return;
            }
            avgXPositions[i] /= (s_hystheresisCount - skipped);
            /*
            bool reCalculate = false;
            for (unsigned short j = 0; j < s_hystheresisCount; j++) {

                if (s_hystheresisArray[j][i].x != 0 && s_hystheresisArray[j][i].x * 5 < avg) {
                    reCalculate = true;
                    break;
                }
            }

            if (reCalculate && attempts < 5) {
                std::cout << "Recalculating...\n";
                i--;
                attempts++;
                continue;
            } else {
                attempts = 0;
            }
            */
            int diff = avgXPositions[i] - previousRow[i].x;
            if (diff > maxDiff[i])
            {
                avgXPositions[i] = previousRow[i].x + maxDiff[i];
            }
            else if (diff < -1 * maxDiff[i])
            {
                avgXPositions[i] = previousRow[i].x - maxDiff[i];
            }
        }

        s_hystheresisArray[s_hystheresisArrayCounter][0] = cv::Point(avgXPositions[0], lowerY);
        s_hystheresisArray[s_hystheresisArrayCounter][1] = cv::Point(avgXPositions[1], upperY);
        s_hystheresisArray[s_hystheresisArrayCounter][2] = cv::Point(avgXPositions[2], lowerY);
        s_hystheresisArray[s_hystheresisArrayCounter][3] = cv::Point(avgXPositions[3], upperY);
    }

#endif

    s_boundaries[0] = s_hystheresisArray[s_hystheresisArrayCounter][0];
    s_boundaries[1] = s_hystheresisArray[s_hystheresisArrayCounter][1];
    s_boundaries[3] = s_hystheresisArray[s_hystheresisArrayCounter][2];
    s_boundaries[2] = s_hystheresisArray[s_hystheresisArrayCounter][3];

    s_hystheresisArrayCounter++;
    if (s_hystheresisArrayCounter == s_hystheresisCount)
    {
        s_hystheresisArrayCounter = 0;
        s_hystheresisArrayFilled = true;
    }
}

void LaneDetection::errorHandler()
{
    std::cerr << "An error has occured!\n";

    try
    {
        std::cerr << "Frame size: " << s_frame.cols << "x" << s_frame.rows << "\n";
        int nonZeroFrame = cv::countNonZero((s_frame.channels() == 1) ? s_frame : cv::Mat());
        if (s_frame.channels() != 1)
        {
            cv::Mat gray;
            cv::cvtColor(s_frame, gray, cv::COLOR_BGR2GRAY);
            nonZeroFrame = cv::countNonZero(gray);
        }
        std::cerr << "Non-zero pixels in processed frame: " << nonZeroFrame << "\n";
        std::cerr << "Mask present: " << (s_mask.empty() ? "no" : "yes") << "\n";
        if (!s_mask.empty())
        {
            std::cerr << "Non-zero pixels in mask: " << cv::countNonZero(s_mask) << "\n";
        }
        std::cerr << "Hough lines detected: " << s_lines.size() << "\n";

        // Save debugging artifacts to disk with timestamp
        std::time_t t = std::time(nullptr);
        std::string ts = std::to_string(static_cast<long long>(t));
        try
        {
            if (!s_frame.empty())
            {
                cv::imwrite(std::string("debug/input/debug_frame_") + ts + ".png", s_frame);
                std::cerr << "Wrote debug/input/debug_frame_" << ts << ".png\n";
            }
            if (!s_mask.empty())
            {
                cv::imwrite(std::string("debug/mask/debug_mask_") + ts + ".png", s_mask);
                std::cerr << "Wrote debug/mask/debug_mask_" << ts << ".png\n";
            }
        }
        catch (const cv::Exception &e)
        {
            std::cerr << "Failed writing debug images: " << e.what() << "\n";
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error handler failure: " << e.what() << "\n";
    }
}

void LaneDetection::prepare(const cv::Size &frameSize, double frameFormat)
{
    createMask(frameSize, frameFormat);
    s_frameCenter = frameSize.width / 2;
    s_maxLineHeight = static_cast<int>(0.66f * frameSize.height);
    s_previewEnabled = hasDisplayServer();

    // Ensure debug folders exist
    ensureDebugDirectories();

    if (!s_previewEnabled)
    {
        std::cerr << "No display server detected, disabling preview window.\n";
    }
}

void LaneDetection::setFrame(const cv::Mat &frame)
{
    s_frame = frame;
}

void LaneDetection::process(cv::Mat &frame)
{
    // Generate timestamp for debug images
    std::time_t t = std::time(nullptr);
    std::stringstream ss;
    ss << std::setfill('0') << std::setw(10) << static_cast<long long>(t);
    std::string ts = ss.str();

    std::cout << "\n=== Lane Detection Pipeline Debug ===";
    std::cout << "\nStarting process() with frame size: " << frame.cols << "x" << frame.rows << "\n";

    setFrame(frame);
    std::cout << "[1] Original image loaded\n";

    edgeDetection(); // detect bright white pixels + clean mask
    std::cout << "[2] Edge detection stages saved to debug/... subfolders\n";

    applyMask(); // crop/ROI
    std::cout << "[3] ROI mask applied - saving to debug/roi/01_roi_" << ts << ".png\n";
    cv::imwrite(std::string("debug/roi/01_roi_") + ts + ".png", s_frame);

    houghLines(); // detect line segments
    std::cout << "[4] Line segments detected: " << s_lines.size() << " lines\n";

    // Visualize detected line segments for debugging
    if (!s_lines.empty())
    {
        cv::Mat houghVis = frame.clone();
        for (const auto &line : s_lines)
        {
            cv::line(houghVis, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]),
                     cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
        }
        std::cout << "    Saving line segments visualization to debug/hough/03_line_segments_" << ts << ".png\n";
        cv::imwrite(std::string("debug/hough/03_line_segments_") + ts + ".png", houghVis);
    }

    if (!s_lines.empty())
    {
        classifyLines(); // filter lane lines (left/right classification)
        std::cout << "[5] Lane lines filtered - Left points: " << s_leftLinePoints.size()
                  << ", Right points: " << s_rightLinePoints.size() << "\n";

        if (s_leftLinePoints.empty() || s_rightLinePoints.empty())
        {
            std::cout << "[X] ERROR: Not enough classified lane points!\n";
            errorHandler();
            return;
        }

        leastSquaresRegression(); // calculate lane regression
        std::cout << "[6] Least squares regression complete\n";
        std::cout << "    Boundary points:\n";
        std::cout << "      Left lower:  (" << s_boundaries[0].x << ", " << s_boundaries[0].y << ")\n";
        std::cout << "      Left upper:  (" << s_boundaries[1].x << ", " << s_boundaries[1].y << ")\n";
        std::cout << "      Right lower: (" << s_boundaries[3].x << ", " << s_boundaries[3].y << ")\n";
        std::cout << "      Right upper: (" << s_boundaries[2].x << ", " << s_boundaries[2].y << ")\n";

        computeLaneCenter();
        computeSteeringError();
        std::cout << "Lane center: " << s_laneCenter << "\n";
        std::cout << "Steering error: " << s_steeringError << "\n";

        display(frame);
        std::cout << "[7] Display overlay applied\n";
        std::cout << "    Saving final output to debug/final/04_output_" << ts << ".png\n";
        cv::imwrite(std::string("debug/final/04_output_") + ts + ".png", frame);
    }
    else
    {
        std::cout << "[X] ERROR: No lines detected!\n";
        errorHandler();
    }

    std::cout << "=== Pipeline Complete ===\n";
}

void LaneDetection::display(cv::Mat &frame)
{

    cv::Mat output;
    frame.copyTo(output);

    // create semi-transparent trapezoid
    cv::fillConvexPoly(output, s_boundaries.data(), 4, cv::Scalar(255, 255, 255), cv::LINE_AA, 0);
    cv::addWeighted(output, 0.4, frame, 0.6, 0, frame);

    // draw left & right lane
    cv::line(frame, s_boundaries[0], s_boundaries[1], cv::Scalar(255, 255, 255), 7, cv::LINE_AA);
    cv::line(frame, s_boundaries[2], s_boundaries[3], cv::Scalar(255, 255, 255), 7, cv::LINE_AA);

    if (!s_previewEnabled)
    {
        return;
    }

    try
    {
        // Display processed frame when GUI backend is available.
        cv::imshow("Lane detection", frame);
        cv::waitKey(1);
    }
    catch (const cv::Exception &e)
    {
        std::cerr << "Preview disabled: " << e.what() << "\n";
        s_previewEnabled = false;
    }
}
