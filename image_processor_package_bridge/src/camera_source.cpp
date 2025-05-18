#include "image_processor_package/camera_source.hpp"

CameraSource::CameraSource(std::shared_ptr<rclcpp::Node> node) // nodeは非コピー，非ムーブ，非参照．subscription作成時に生きている必要があるのでshared_ptr
{
    subscription_ = node->create_subscription<sensor_msgs::msg::Image>(
        "/camera/camera/color/image_raw", 10,
        std::bind(&CameraSource::callback, this, std::placeholders::_1));
}

void CameraSource::callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    latest_frame_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
    has_frame_ = true;
}

std::optional<cv::Mat> CameraSource::get_frame()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (has_frame_)
    {
        has_frame_ = false;
        return latest_frame_.clone();
    }
    else
    {
        return std::nullopt;
    }
}