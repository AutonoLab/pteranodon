#pragma once

#include <vector>
#include <opencv2/opencv.hpp>

class BlobDetector 
{
public:
    BlobDetector(bool filter_blobs = true, bool merge_blobs = false);
    virtual ~BlobDetector();

    std::vector<cv::Rect> detect(cv::Mat& image);
    cv::Rect detectAnchor(cv::Mat& image, cv::Rect& anchor);
    float getBestScore() const { return best_score; }

    // operators
    friend std::ostream& operator<<(std::ostream& t_out, const BlobDetector& t_detector);

private:
    // configuration
    const bool m_filter_blobs;
    const bool m_merge_blobs;

    float best_score = 0.0f;

    // detection methods
    void preprocess(cv::Mat& t_image) const;
    std::vector<cv::Rect> generateBlobs(cv::Mat& t_image) const;
    void filterBlobs(cv::Mat& t_image, std::vector<cv::Rect>& t_blobs) const;
    void mergeBlobs(cv::Mat& t_image, std::vector<cv::Rect>& t_blobs) const;
    std::vector<float> scoreBlobs(cv::Mat& t_image, cv::Rect& t_anchor, std::vector<cv::Rect>& t_blobs) const;

    // utility methods
    static int compareRects(cv::Rect& t_rect1, cv::Rect& t_rect2) {
        return t_rect1.size().area() - t_rect2.size().area();
    }
};
