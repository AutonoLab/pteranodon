#include <pteranodon_ext>

#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

int main() {
    cv::Mat image = cv::imread("../data/test.png");
    cv::Rect anchor = cv::Rect(0, 0, 100, 100);
    BlobDetector detector = BlobDetector();
    
    // detect with an anchor box
    cv::Rect blob = detector.detect(image, anchor);
    std::cout << blob << std::endl;

    // detect without an anchor box
    std::vector<cv::Rect> blobs = detector.detect(image);
    for (auto blob : blobs) {
        std::cout << blob << std::endl;
    }

    return 0;
}
