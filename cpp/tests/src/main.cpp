#include <pteranodon_ext>

#include <vector>
#include <iostream>

#include <opencv2/opencv.hpp>

int main() {
    cv::Mat image = cv::imread("data/test.png");
    cv::Rect anchor = cv::Rect(0, 0, 100, 100);
    BlobDetector detector(true, false);
    
    // detect with an anchor box
    cv::Rect blob = detector.detectAnchor(image, anchor);
    std::cout << "Results from detect with anchor: " << std::endl;
    std::cout << blob << std::endl;

    // detect without an anchor box
    std::vector<cv::Rect> blobs = detector.detect(image);
    std::cout << "Results from detect without anchor: " << std::endl;
    for (auto blob : blobs) {
        std::cout << blob << std::endl;
    }

    return 0;
}
