#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>

#include "LaneDetection.h"
#include "Timer.h"
#include "lane_pipe_writer.h"

namespace {

    const char *kDefaultCameraPipeline = "v4l2src device=/dev/video0 ! "
                                         "video/x-raw,format=NV12,width=640,height=480 ! "
                                         "videoconvert ! "
                                         "video/x-raw,format=BGR ! "
                                         "appsink";

    bool isPipelineSource(const std::string &source)
    {
        return source.find("v4l2src") != std::string::npos
            || source.find("appsink") != std::string::npos
            || source.find('!') != std::string::npos;
    }

    bool isDigitsOnly(const std::string &value)
    {
        return !value.empty()
            && std::all_of(value.begin(), value.end(),
                           [](unsigned char c) { return std::isdigit(c) != 0; });
    }

    std::string toLower(std::string value)
    {
        std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
            return static_cast<char>(std::tolower(c));
        });
        return value;
    }

    bool hasExtension(const std::string &source,
                      const std::initializer_list<const char *> &exts)
    {
        auto dot = source.find_last_of('.');
        if (dot == std::string::npos) {
            return false;
        }

        const std::string ext = toLower(source.substr(dot));
        for (const char *expected : exts) {
            if (ext == expected) {
                return true;
            }
        }
        return false;
    }

    bool isImagePath(const std::string &source)
    {
        return hasExtension(source,
                            {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff", ".webp"});
    }

    bool isVideoPath(const std::string &source)
    {
        return hasExtension(
            source, {".mp4", ".avi", ".mov", ".mkv", ".webm", ".m4v", ".mpg", ".mpeg"});
    }

    int runImageMode(const std::string &source)
    {
        cv::Mat frame;
        frame = cv::imread(source, cv::IMREAD_COLOR);

        // error loading image
        if (frame.empty()) {
            std::cerr << "Failed to load image: " << source << "\n";
            return 1;
        }

        // resize the provided image to 640 x 480
        cv::resize(frame, frame, cv::Size(640, 480));

        std::cout << "Image loaded successfully: " << source << "\n";

        // prepare image and run through lane detection pipeline
        LaneDetection::prepare(frame.size(), frame.type());
        LaneDetection::process(frame);

        // Keep window open for image mode if preview is enabled.
        cv::waitKey(0);
        return 0;
    }

} // namespace

int main(int argc, const char **argv)
{
    // Use default GStreamer source if no source provided
    std::string source = kDefaultCameraPipeline;

    // Use command line source if specified
    if (argc > 1) {
        source = argv[1];
    }

    // cv::Mat frame;

    if (isImagePath(source)) {
        return runImageMode(source);
    }

    cv::VideoCapture cap;
    bool opened = false;
    try {
        if (isPipelineSource(source)) {
            opened = cap.open(source, cv::CAP_GSTREAMER);
        } else if (isVideoPath(source)) {
            opened = cap.open(source);
        } else if (isDigitsOnly(source)) {
            opened = cap.open(std::atoi(source.c_str()));
        } else {
            // Fallback: try opening as file/URL first, then as camera index.
            opened = cap.open(source);
            if (!opened && cap.isOpened()) {
                cap.release();
            }
            if (!opened) {
                opened = cap.open(std::atoi(source.c_str()));
            }
        }
    } catch (const cv::Exception &e) {
        std::cerr << "OpenCV threw while opening source: " << e.what() << "\n";
        return 2;
    }

    if (!opened || !cap.isOpened()) {
        std::cerr << "Failed to open source: " << source << "\n";
        return 1;
    }

    std::cout << "Source opened successfully: " << source << "\n";

    // if (!cap.read(frame) || frame.empty()) {
    //     std::cerr << "Failed to read first frame from source.\n";
    //     return 3;
    // }
}
