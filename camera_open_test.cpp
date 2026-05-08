#include <opencv2/opencv.hpp>

#include <cstdlib>
#include <iostream>
#include <string>

int main(int argc, char** argv) {
    std::string source = "v4l2src device=/dev/video0 ! video/x-raw,format=NV12 ! videoconvert ! video/x-raw,format=BGR ! appsink";
    if (argc > 1) {
        source = argv[1];
    }

    std::string outputPath = "camera_frame.png";
    if (argc > 2) {
        outputPath = argv[2];
    }

    // Optional image tuning for dark captures:
    // alpha: contrast multiplier (1.0 = unchanged)
    // beta: brightness offset in pixel values (0 = unchanged)
    double alpha = 1.0;
    double beta = 0.0;
    if (argc > 3) {
        alpha = std::atof(argv[3]);
    }
    if (argc > 4) {
        beta = std::atof(argv[4]);
    }

    cv::VideoCapture capture;

    bool opened = false;
    try {
        if (source.find("v4l2src") != std::string::npos ||
            source.find("appsink") != std::string::npos ||
            source.find('!') != std::string::npos) {
            opened = capture.open(source, cv::CAP_GSTREAMER);
        } else {
            opened = capture.open(std::atoi(source.c_str()));
        }
    } catch (const cv::Exception& error) {
        std::cerr << "OpenCV threw while opening camera: " << error.what() << '\n';
        return 2;
    }

    if (!opened || !capture.isOpened()) {
        std::cerr << "Camera failed to open: " << source << '\n';
        return 1;
    }

    std::cout << "Camera opened successfully: " << source << '\n';

    cv::Mat frame;

    // Let auto-exposure/auto-white-balance settle before saving.
    const int warmupFrames = 20;
    for (int i = 0; i < warmupFrames; ++i) {
        if (!capture.read(frame) || frame.empty()) {
            std::cerr << "Camera opened, but frame capture failed during warmup.\n";
            return 3;
        }
    }

    if (!capture.read(frame) || frame.empty()) {
        std::cerr << "Camera opened, but frame capture failed.\n";
        return 3;
    }

    if (alpha != 1.0 || beta != 0.0) {
        cv::Mat adjusted;
        frame.convertTo(adjusted, -1, alpha, beta);
        frame = adjusted;
    }

    std::cout << "Frame captured: " << frame.cols << 'x' << frame.rows << '\n';
    std::cout << "Image tuning used alpha=" << alpha << ", beta=" << beta << '\n';

    if (!cv::imwrite(outputPath, frame)) {
        std::cerr << "Failed to write image: " << outputPath << '\n';
        return 4;
    }

    std::cout << "Saved frame to: " << outputPath << '\n';
    return 0;
}