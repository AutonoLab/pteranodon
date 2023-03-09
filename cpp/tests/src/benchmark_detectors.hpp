#include <pteranodon_ext>

#include <opencv2/opencv.hpp>

#include <iostream>
#include <vector>
#include <chrono>

int benchmarkDetectors() {
    BlobDetector blobDetectNone(false, false);
    BlobDetector blobDetectFilter(true, false);
    // BlobDetector blobDetectMerge(false, true);
    // BlobDetector blobDetectBoth(true, true);

    // std::vector<BlobDetector> detectors{blobDetectNone, blobDetectFilter, blobDetectMerge, blobDetectBoth};
    std::vector<BlobDetector> detectors{blobDetectNone, blobDetectFilter};
    std::vector<int> detectTimes;
    std::vector<int> detectAnchorTimes;

    for(auto detector : detectors)
    {
        detectTimes.push_back(0);
        detectAnchorTimes.push_back(0);
    }

    // // iterate over decorators with i and print it
    // for(int i = 0; i < detectors.size(); i++){
    //     std::cout << "Detector " << i << ": " << detectors[i] << std::endl;
    // }

    // load a video
    cv::VideoCapture cap("depth_1.mp4");

    // check if video is loaded
    if (!cap.isOpened()) {
        std::cout << "Error opening video stream or file" << std::endl;
        return -1;
    }

    // get the first frame
    cv::Mat frame;

    auto anchor = cv::Rect(50, 50, 50, 50);

    int counter = 0;

    std::cout << "Starting benchmark..." << std::endl;

    std::cout << "   Iterating over " << detectors.size() << " detectors..." << std::endl;

    // iterate over loading frames
    while (1) {
        // load the next frame
        cap >> frame;

        // check if frame is empty
        if (frame.empty())
            break;

        // run detect and detectAnchor on each detector in the vector detectors
        // store the time it takes to run each detector
        for(int i = 0; i < detectors.size(); i++){
            auto start = std::chrono::high_resolution_clock::now();
            detectors[i].detect(frame);
            auto stop = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
            detectTimes[i] += duration.count();
        }

        for(int i = 0; i < detectors.size(); i++){
            auto start = std::chrono::high_resolution_clock::now();
            detectors[i].detectAnchor(frame, anchor);
            auto stop = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
            detectAnchorTimes[i] += duration.count();
        }

        counter++;
    }

    // print the average times
    std::cout << "Average times detect on " << counter << " frames:" << std::endl;
    std::cout << "BlobDetector(false, false): " << detectTimes[0] / counter << "ms" << std::endl;
    std::cout << "BlobDetector(true, false): " << detectTimes[1] / counter << "ms" << std::endl;
    std::cout << "BlobDetector(false, true): " << detectTimes[2] / counter << "ms" << std::endl;
    std::cout << "BlobDetector(true, true): " << detectTimes[3] / counter << "ms" << std::endl << std::endl;
    std::cout << "Average times detectAnchor on " << counter << " frames:" << std::endl;
    std::cout << "BlobDetector(false, false): " << detectAnchorTimes[0] / counter << "ms" << std::endl;
    std::cout << "BlobDetector(true, false): " << detectAnchorTimes[1] / counter << "ms" << std::endl;
    std::cout << "BlobDetector(false, true): " << detectAnchorTimes[2] / counter << "ms" << std::endl;
    std::cout << "BlobDetector(true, true): " << detectAnchorTimes[3] / counter << "ms" << std::endl;


    return 0;

}