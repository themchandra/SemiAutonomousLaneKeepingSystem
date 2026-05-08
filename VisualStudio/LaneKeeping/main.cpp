#include <iostream>
#include <opencv2/opencv.hpp>
#include <cstdlib>
#include <string>

#include "LaneDetection.h"
#include "Timer.h"

namespace {

const char* kDefaultCameraPipeline =
    "v4l2src device=/dev/video0 ! "
    "video/x-raw,format=NV12,width=640,height=480 ! "
    "videoconvert ! "
    "video/x-raw,format=BGR ! "
    "appsink";

bool isPipelineSource(const std::string& source) {
    return source.find("v4l2src") != std::string::npos
        || source.find("appsink") != std::string::npos
        || source.find('!') != std::string::npos;
}

} // namespace

int main(int argc, const char **argv)
{
    std::string source = kDefaultCameraPipeline;
    if (argc > 1) {
        source = argv[1];
    }

    cv::VideoCapture cap;
    bool opened = false;
    try {
        if (isPipelineSource(source)) {
            opened = cap.open(source, cv::CAP_GSTREAMER);
        } else {
            opened = cap.open(std::atoi(source.c_str()));
        }
    } catch (const cv::Exception& e) {
        std::cerr << "OpenCV threw while opening camera: " << e.what() << "\n";
        return 2;
    }

    if (!opened || !cap.isOpened()) {
        std::cerr << "Failed to open camera source: " << source << "\n";
        return 1;
    }

    std::cout << "Camera opened successfully: " << source << "\n";

    cv::Mat frame;
    const int warmupFrames = 20;
    for (int i = 0; i < warmupFrames; ++i) {
        if (!cap.read(frame) || frame.empty()) {
            std::cerr << "Failed to read frame during warmup.\n";
            return 3;
        }
    }

    if (!cap.read(frame) || frame.empty()) {
        std::cerr << "Failed to read first frame from camera.\n";
        return 3;
    }

    // Use the actual frame metadata from a valid capture frame.
    LaneDetection::prepare(frame.size(), frame.type());

    long long frameCount = static_cast<long long>(cap.get(cv::CAP_PROP_FRAME_COUNT));
    if (frameCount < 0) {
        frameCount = 0;
    }

    auto globalTimer = new Timer("Loop", frameCount);

    do {
        auto t = new Timer("Loop");
        LaneDetection::process(frame);
        delete t;
    } while (cap.read(frame));

    delete globalTimer;
}
