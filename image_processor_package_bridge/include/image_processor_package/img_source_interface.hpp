#ifndef IMAGE_PROCESSOR_PACKAGE__IMG_SOURCE_INTERFACE_HPP_
#define IMAGE_PROCESSOR_PACKAGE__IMG_SOURCE_INTERFACE_HPP_

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/highgui.hpp>

#include "sensor_msgs/msg/image.hpp"
#include <optional>

class ImgSource
{
public:
    virtual ~ImgSource() = default;
    virtual std::optional<cv::Mat> get_frame() = 0;
};

#endif