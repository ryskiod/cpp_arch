#include "image_processor_package/mock_source.hpp"
#include <opencv2/imgproc.hpp>

std::optional<cv::Mat> MockSource::get_frame()
{
    if (already_sent_)
        return std::nullopt;

    already_sent_ = true;

    cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC3);
    cv::putText(image, "Mock Frame", cv::Point(100, 240), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
    return image;
}