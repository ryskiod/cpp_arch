#include "image_processor_package/img_subscriber.hpp"
#include "image_processor_package/processor_factory.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    std::string type{"passthrough"};

    auto processor = ProcessorFactory::create(type);
    auto node = std::make_shared<ImgSubscriber>(std::move(processor));

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}