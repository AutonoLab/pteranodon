#include "../include/detectors/blob_detector.hpp"

#include <iterator>

BlobDetector::BlobDetector(bool filter_blobs, bool merge_blobs) :
    m_filter_blobs(filter_blobs),
    m_merge_blobs(merge_blobs)
{
}

BlobDetector::~BlobDetector()
{
}

std::vector<cv::Rect> BlobDetector::detect(cv::Mat& t_image)
{
    preprocess(t_image);
    std::vector<cv::Rect> blobs = generateBlobs(t_image);
    if (m_filter_blobs) {
        filterBlobs(t_image, blobs);
    }
    if (m_merge_blobs) {
        mergeBlobs(t_image, blobs);
    }
    return blobs;
}

cv::Rect BlobDetector::detect(cv::Mat& t_image, cv::Rect& t_anchor)
{
    preprocess(t_image);
    std::vector<cv::Rect> blobs = generateBlobs(t_image);
    if (m_filter_blobs) {
        filterBlobs(t_image, blobs);
    }
    if (m_merge_blobs) {
        mergeBlobs(t_image, blobs);
    }
    std::vector<float> scores = scoreBlobs(t_image, t_anchor, blobs);

    // get the index of the highest score
    int max_index = std::distance(scores.begin(), std::max_element(scores.begin(), scores.end()));

    return blobs[max_index];
}

void BlobDetector::preprocess(cv::Mat& t_image) const
{
    // convert the image to grayscale
    cv::cvtColor(t_image, t_image, cv::COLOR_BGR2GRAY);

    // apply a gaussian blur to the image
    cv::GaussianBlur(t_image, t_image, cv::Size(5, 5), 0);

    // perform a threshold on the image
    cv::threshold(t_image, t_image, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
}

std::vector<cv::Rect> BlobDetector::generateBlobs(cv::Mat& t_image) const
{
    // find the contours in the image
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(t_image, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    // create a vector of bounding boxes
    std::vector<cv::Rect> blobs;

    // loop over the contours
    for (auto& contour : contours)
    {
        // create a bounding box for the contour
        cv::Rect bounding_box = cv::boundingRect(contour);

        // add the bounding box to the vector
        blobs.push_back(bounding_box);
    }

    return blobs;
}

void BlobDetector::filterBlobs(cv::Mat& t_image, std::vector<cv::Rect>& t_blobs) const
{
    // create a vector to hold the filtered blobs
    std::vector<cv::Rect> filtered_blobs;

    // get the height and width of the image
    int i_height = t_image.rows;
    int i_width = t_image.cols;

    // get the area of the overall image
    int i_area = i_height * i_width;

    // loop over the blobs
    for (auto& blob : t_blobs)
    {
        // get the width and height of the blob
        int b_width = blob.width;
        int b_height = blob.height;

        // get the area of the blob
        int b_area = width * height;

        // ratio of the blob area to the image area
        float area_ratio = (float)b_area / (float)i_area;

        // check if the area_ratio is small enough (i.e. the blob is not the entire image)
        if (area_ratio < 0.90)
        {
            continue;
        }
        filtered_blobs.push_back(blob);
    }

    // replace the blobs with the filtered blobs
    t_blobs = filtered_blobs;
}

void BlobDetector::mergeBlobs(cv::Mat& t_image, std::vector<cv::Rect>& t_blobs) const
{
    // create a vector to hold the merged blobs
    std::vector<cv::Rect> merged_blobs;

    // sort the blobs by size
    std::sort(t_blobs.begin(), t_blobs.end(), compareRects);

    for (int i = 0; i < t_blobs.size(); i++) {
        for (int j = i; j < t_blobs.size(); j++) {
            // get the current blob
            cv::Rect blob1 = t_blobs[i];

            // get the next blob
            cv::Rect blob2 = t_blobs[j];

            if (blob1 == blob2)
            {
                continue;
            }

            // check if blob2 (the smaller one) is contained inside of blob1
            // and the sizes are roughly the same
            if (blob1.contains(blob2.tl()) && 
                blob1.contains(blob2.br()) &&
                blob1.size().area() / blob2.size().area() < 1.5)
            {
                // merge the blobs
                cv::Rect merged_blob = blob1 | blob2;

                // add the merged blob to the vector
                merged_blobs.push_back(merged_blob);
            }
        }
    }

    // replace the blobs with the merged blobs
    t_blobs = merged_blobs;
}

std::vector<float> BlobDetector::scoreBlobs(cv::Mat& t_image, cv::Rect& t_anchor, std::vector<cv::Rect>& t_blobs) const
{
    // create a vector to hold the scores
    std::vector<float> scores;

    // get the center of the anchor
    cv::Point anchor_center = (t_anchor.tl() + t_anchor.br()) * 0.5;

    // get the area of the anchor
    int anchor_area = t_anchor.area();

    // loop over the blobs
    for (auto& blob : t_blobs)
    {
        // get the center of the blob
        cv::Point blob_center = (blob.tl() + blob.br()) * 0.5;

        // get the distance between the anchor and the blob
        float distance = cv::norm(anchor_center - blob_center);

        // get the area of the blob
        float area = blob.area();

        // calculate the score
        float area_diff = (float)anchor_area / (float)area;

        if (area_diff < 0.9 || area_diff > 1.1)
        {
            continue;
        }

        if (distance > 100)
        {
            continue;
        }

        // add the score to the vector
        scores.push_back(score);
    }

    return scores;
}

int BlobDetector::compareRects(cv::Rect& t_rect1, cv::Rect& t_rect2) const
{
    return t_rect1.size() - t_rect2.size();
}
