#include "image_processor_package/img_subscriber.hpp"

ImgSubscriber::ImgSubscriber(std::unique_ptr<IProcessor> processor) : Node("img_subscriber"),
                                                                      processor_(std::move(processor))
{
    // [this](const sensor_msgs::msg::Image::SharedPtr msg)
    // {
    //     topic_callback(msg);
    // }
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/camera/color/image_raw", 10,
        std::bind(&ImgSubscriber::topic_callback, this, std::placeholders::_1));
}

void ImgSubscriber::topic_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    cv::Mat cv_image = cv_bridge::toCvCopy(msg, "bgr8")->image;
    cv::Mat processed_image{processor_->process(cv_image)};
    // strategy pattern
    // processorの具体的な中身を知らなくても良くし，processorの切り替えをなくし，processorの追加・変更からの影響を少なくした．

    cv::imshow("Received Image", processed_image);
    cv::waitKey(1);
}