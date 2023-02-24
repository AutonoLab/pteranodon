#ifndef TMP_BLOB_DETECTOR_H_
#define TMP_BLOB_DETECTOR_H_

#include <vector>
#include <opencv2/core.hpp>

class BlobDetector 
{
public:
    BlobDetector();
    ~BlobDetector();

    std::vector<cv::Rect> detect(cv::Mat& t_image);
    cv::Rect detect(cv::Mat& t_image, cv::Rect& t_anchor);

private:
    // detection methods
    void preprocess(cv::Mat& t_image) const;
    std::vector<cv::Rect> generateBlobs(cv::Mat& t_image) const;
    void filterBlobs(cv::Mat& t_image, std::vector<cv::Rect>& t_blobs) const;
    void mergeBlobs(cv::Mat& t_image, std::vector<cv::Rect>& t_blobs) const;
    std::vector<float> scoreBlobs(cv::Mat& t_image, cv::Rect& t_anchor, std::vector<cv::Rect>& t_blobs) const;

    // utility methods
    int compareRects(cv::Rect& t_rect1, cv::Rect& t_rect2) const;
}

#endif  // TMP_BLOB_DETECTOR_H_
