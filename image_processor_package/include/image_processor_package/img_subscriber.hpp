#ifndef IMAGE_PROCESSOR_PACKAGE__IMG_SUBSCRIBER_HPP_
#define IMAGE_PROCESSOR_PACKAGE__IMG_SUBSCRIBER_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/highgui.hpp>

#include "sensor_msgs/msg/image.hpp"
#include "image_processor_package/processor_interface.hpp"

class ImgSubscriber : public rclcpp::Node
{
public:
    explicit ImgSubscriber(std::unique_ptr<IProcessor> processor);

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    std::unique_ptr<IProcessor> processor_;

    void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg);
};

#endif // IMAGE_PROCESSOR_PACKAGE__IMG_SUBSCRIBER_HPP_