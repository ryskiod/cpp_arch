#ifndef IMAGE_PROCESSOR_PACKAGE__IMG_SUBSCRIBER_HPP_
#define IMAGE_PROCESSOR_PACKAGE__IMG_SUBSCRIBER_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/highgui.hpp>

#include "sensor_msgs/msg/image.hpp"
#include "type_erasure_processor.hpp"

class ImgSubscriber : public rclcpp::Node
{
public:
    // explicit ImgSubscriber(std::unique_ptr<IProcessor> processor);
    explicit ImgSubscriber(TypeErasedProcessor processor); // functionでポインタではなく，値として扱うようになったためunique_ptrでなくなる

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    TypeErasedProcessor processor_;

    void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg);
};

#endif // IMAGE_PROCESSOR_PACKAGE__IMG_SUBSCRIBER_HPP_