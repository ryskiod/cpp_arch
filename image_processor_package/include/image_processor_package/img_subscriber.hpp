#ifndef IMAGE_PROCESSOR_PACKAGE__IMG_SUBSCRIBER_HPP_
#define IMAGE_PROCESSOR_PACKAGE__IMG_SUBSCRIBER_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/highgui.hpp>

#include "sensor_msgs/msg/image.hpp"
#include "image_processor_package/processor_interface.hpp" //ここで具体的なprocessroを知らなくて良い

class ImgSubscriber : public rclcpp::Node
{
public:
    explicit ImgSubscriber(std::unique_ptr<IProcessor> processor);
    // explicit: 暗黙の型変換を防ぎIProcessorの意図的ではないmove, 所有権の譲渡を防ぐため

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    std::unique_ptr<IProcessor> processor_;

    void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg);
};

#endif // IMAGE_PROCESSOR_PACKAGE__IMG_SUBSCRIBER_HPP_