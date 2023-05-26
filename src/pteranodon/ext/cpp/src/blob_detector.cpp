#include "../include/headers/blob_detector.hpp"

#include <iterator>

BlobDetector::BlobDetector(bool filter_blobs, bool merge_blobs) :
    m_filter_blobs(filter_blobs),
    m_merge_blobs(merge_blobs)
{
}

BlobDetector::~BlobDetector()
{
}

std::vector<cv::Rect> BlobDetector::detect(cv::Mat& image)
{
    // store the original image since we will be modifying it
    cv::Mat saved_image = image.clone();
    
    preprocess(image);

    std::vector<cv::Rect> blobs = generateBlobs(image);

    if (m_filter_blobs) {
        filterBlobs(image, blobs);
    }
    if (m_merge_blobs) {
        mergeBlobs(image, blobs);
    }

    // restore the original image
    image = saved_image;

    return blobs;
}

cv::Rect BlobDetector::detectAnchor(cv::Mat& image, cv::Rect& anchor)
{
    // store the original image since we will be modifying it
    cv::Mat saved_image = image.clone();

    preprocess(image);
    std::vector<cv::Rect> blobs = generateBlobs(image);
    if (m_filter_blobs) {
        filterBlobs(image, blobs);
    }
    if (m_merge_blobs) {
        mergeBlobs(image, blobs);
    }
    std::vector<float> scores = scoreBlobs(image, anchor, blobs);

    // get the index of the highest score
    int max_index = std::distance(scores.begin(), std::max_element(scores.begin(), scores.end()));

    // save the best score
    best_score = scores[max_index];

    // restore the original image
    image = saved_image;

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

    // perform findContours
    cv::findContours(t_image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

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
        int b_area = b_width * b_height;

        // ratio of the blob area to the image area
        float area_ratio = (float)b_area / (float)i_area;

        // check if the area_ratio is small enough (i.e. the blob is not the entire image)
        if (area_ratio > 0.90)
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

    // iterate over the blobs
    // if the current blob contains the next blob, merge them
    for (int i = 0; i < t_blobs.size(); i++){
        auto bigBlob = t_blobs[i];
        // iterate over the blobs remaining in the vector excluding the current blob
        for (int j = i + 1; j < t_blobs.size(); j++){
            auto smallBlob = t_blobs[j];
            // check if the current blob contains the next blob
            bool contains = bigBlob.contains(smallBlob.tl()) && bigBlob.contains(smallBlob.br());

            // check if the blobs are roughly the same size
            bool same_size = (smallBlob.area() / bigBlob.area()) > 0.9;

            // if the current blob contains the next blob, merge them
            if (contains && same_size){
                // get the top left and bottom right points of the merged blob
                cv::Point tl = cv::Point(std::min(bigBlob.tl().x, smallBlob.tl().x), std::min(bigBlob.tl().y, smallBlob.tl().y));
                cv::Point br = cv::Point(std::max(bigBlob.br().x, smallBlob.br().x), std::max(bigBlob.br().y, smallBlob.br().y));

                // create a new blob from the top left and bottom right points
                cv::Rect merged_blob = cv::Rect(tl, br);

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

        // calculate the difference in area
        float area_diff = (float)anchor_area / (float)area;

        // compute the score
        float score = area_diff * distance;

        // add the score to the vector
        scores.push_back(score);
    }

    // normalize the scores againist the maximum score
    // then invert the scores, to become bigger is better metric
    float max_score = *std::max_element(scores.begin(), scores.end());
    for (auto& score : scores)
    {
        score = 1 - (score / max_score);
    }

    return scores;
}

// operator definitions
std::ostream& operator<<(std::ostream& t_out, const BlobDetector& t_detector){
    t_out << "BlobDetector: " << std::endl;
    t_out << "Filter Blobs: " << t_detector.m_filter_blobs << std::endl;
    t_out << "Merge Blobs: " << t_detector.m_merge_blobs << std::endl;
    return t_out;
}
