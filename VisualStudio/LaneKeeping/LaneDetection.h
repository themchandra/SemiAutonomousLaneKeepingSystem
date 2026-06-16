#pragma once
#include <array>
#include <opencv2/opencv.hpp>
#include <vector>

class LaneDetection {

// Enable a short temporal smoothing buffer to stabilize detection
// across noisy frames. Tweak `s_hystheresisCount` to change how many
// past samples are averaged.
#define USE_HYSTHERESIS

    // Current working frame (may be converted to grayscale during
    // processing).
    static cv::Mat s_frame;
    static int s_frameCenter;
    static int s_maxLineHeight;
    static const unsigned short s_hystheresisCount = 6;
    static std::array<std::array<cv::Point, 4>, s_hystheresisCount> s_hystheresisArray;
    static unsigned short s_hystheresisArrayCounter;
    static bool s_hystheresisArrayFilled;
    static cv::Mat s_mask;

    // Intermediate Hough output (vector of detected line segments)
    static std::vector<cv::Vec4i> s_lines;

    // Points collected for left / right lane candidates. These are used
    // by fitLine() to compute a single line per side.
    static std::vector<cv::Point> s_rightLinePoints;
    static std::vector<cv::Point> s_leftLinePoints;

    // The four points that describe the detected lane polygon used for
    // visualization and blending on the original frame.
    static std::array<cv::Point, 4> s_boundaries;
    static bool s_previewEnabled;
    static int s_laneCenter;
    static int s_steeringError;
    static float s_normalizedSteeringError;

    static void computeLaneCenter();
    static void computeSteeringError();
    static void computeNormalizedSteeringError();

    // Build a lower-image ROI based on expected camera mounting angle.
    static void createMask(const cv::Size &frameSize, double frameFormat);
    inline static void applyMask();
    inline static void changeContrast();
    inline static void blur();

    // Extract likely white lane markings and convert them into an edge image.
    inline static void edgeDetection();
    inline static void houghLines();

    // Split Hough segments into left/right lane candidates.
    static void classifyLines();

    // Fit one boundary line per side from classified candidates.
    static void leastSquaresRegression();
    inline static void hystheresis(std::array<float, 4> xPositions, int lowerY,
                                   int upperY);

    static void errorHandler();

    static bool hasDisplayServer();

  public:
    static void prepare(const cv::Size &frameSize, double frameFormat);
    static void setFrame(const cv::Mat &frame);
    static void process(cv::Mat &frame);
    static void display(cv::Mat &frame);
    static int getSteeringError();
    static float getNormalizedSteeringError();
};
