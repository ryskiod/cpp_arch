#ifndef PROCESSOR_FACTORY_HPP_
#define PROCESSOR_FACTORY_HPP_

#include "processor_interface.hpp"
#include "passthrough_processor.hpp"
#include "img_source_interface.hpp"
#include "camera_source.hpp"
#include "mock_source.hpp"

class ProcessorFactory
{
public:
    static std::unique_ptr<IProcessor> create(const std::string &type)
    {
        if (type == "passthrough")
        {
            return std::make_unique<PassthroughProcessor>();
        }
        // if (type == "canny") //Processorの追加はここのみ
        // {

        // }

        throw std::invalid_argument("unknown processor type: " + type);
    }
};

class SourceFactory
{
public:
    static std::unique_ptr<ImgSource> create(const std::string &type, std::shared_ptr<rclcpp::Node> node)
    {
        if (type == "mock")
        {
            return std::make_unique<MockSource>();
        }
        else if (type == "camera")
        {
            return std::make_unique<CameraSource>(node);
        }

        throw std::invalid_argument("unknown processor type: " + type);
    }
};
#endif