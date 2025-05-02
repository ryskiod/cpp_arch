#ifndef PROCESSOR_FACTORY_HPP_
#define PROCESSOR_FACTORY_HPP_

#include "processor_interface.hpp"
#include "passthrough_processor.hpp"

class ProcessorFactory
{
public:
    static std::unique_ptr<IProcessor> create(const std::string &type)
    {
        if (type == "passthrough")
        {
            return std::make_unique<PassthroughProcessor>();
        }
        // if (type == "canny")
        // {

        // }

        throw std::invalid_argument("unknown processor type: " + type);
    }
};

#endif