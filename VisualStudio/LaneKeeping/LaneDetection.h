// Lightweight lane detection helper
// This class implements a simple lane detection pipeline used by the
// lane-keeping demo. It's structured as a collection of static helpers
// so the detection state is global and easy to call from the single
// application thread.
//
// Key responsibilities:
// - prepare(): initialize mask and runtime parameters for the incoming
//   frame size/format
// - process(): run the full per-frame pipeline (contrast -> blur ->
//   edge detection -> Hough -> classification -> regression)
// - display(): draw detected lane boundaries and (optionally) preview

#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <array>

class LaneDetection
{

// Enable a short temporal smoothing buffer to stabilize detection
// across noisy frames. Tweak `s_hystheresisCount` to change how many
// past samples are averaged.
#define USE_HYSTHERESIS

	// Current working frame (may be converted to grayscale during
	// processing).
	static cv::Mat s_frame;

	// Cached geometry derived from frame size: center x coordinate and
	// maximum y used for line drawing/height calculations.
	static int s_frameCenter;
	static int s_maxLineHeight;

	// Hysteresis (temporal) buffer used to smooth lane boundaries
	// across multiple frames. The array stores up to
	// `s_hystheresisCount` rows of four points (left lower, left upper,
	// right lower, right upper).
	static const unsigned short s_hystheresisCount = 6;
	static std::array<std::array<cv::Point, 4>, s_hystheresisCount> s_hystheresisArray;
	static unsigned short s_hystheresisArrayCounter;
	static bool s_hystheresisArrayFilled;

	// Region-of-interest mask (single-channel) applied to the processed
	// image to ignore irrelevant areas outside the road.
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

	// If false, `display()` will skip `imshow`/`waitKey` calls (useful
	// for headless operation). Determined at runtime from the
	// environment (DISPLAY / WAYLAND_DISPLAY) and flipped off on any
	// cv::Exception from the GUI backend.
	static bool s_previewEnabled;

	// Internal pipeline steps. Most are small helpers and intentionally
	// inline to allow easy tuning without changing external behavior.
	static void createMask(const cv::Size &frameSize, double frameFormat);
	static inline void applyMask();
	static inline void changeContrast();
	static inline void blur();
	static inline void edgeDetection();
	static inline void houghLines();
	static void classifyLines();
	static void leastSquaresRegression();
	static inline void hystheresis(std::array<float, 4> xPositions, int lowerY, int upperY);

	// Called when no valid lines can be produced for the current frame.
	static void errorHanlder();

	// Detect whether an X/Wayland display is available on startup.
	static bool hasDisplayServer();

public:
	// Prepare runtime structures for a given frame size/format. This
	// must be called once with a valid frame size before calling
	// `process()`.
	static void prepare(const cv::Size &frameSize, double frameFormat);

	// Set the current frame (copy or reference depending on caller).
	// `process()` will operate on the last set frame.
	static void setFrame(const cv::Mat &frame);

	// Run the full detection pipeline on the provided frame. After
	// calling `process()` the latest lane boundaries will be available
	// for use by other components or for visualization via `display()`.
	static void process(cv::Mat &frame);

	// Overlay detected lane boundaries on `frame` and (optionally)
	// display a preview window. Safe to call in headless mode.
	static void display(cv::Mat &frame);
};
