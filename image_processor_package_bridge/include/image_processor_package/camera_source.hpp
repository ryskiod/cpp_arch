#ifndef CAMERA_SOURCE_HPP_
#define CAMERA_SOURCE_HPP_

#include "img_source_interface.hpp"

class CameraSource : public ImgSource
{
public:
    CameraSource(std::shared_ptr<rclcpp::Node> node);
    std::optional<cv::Mat> get_frame() override;

private:
    cv::Mat latest_frame_;
    std::mutex mutex_;
    bool has_frame_ = false;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    void callback(const sensor_msgs::msg::Image::SharedPtr msg);
};

#endif