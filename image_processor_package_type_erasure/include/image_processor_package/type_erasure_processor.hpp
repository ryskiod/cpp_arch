#ifndef TYPE_ERASURE_PROCESSOR_INTERFACE_HPP_
#define TYPE_ERASURE_PROCESSOR_INTERFACE_HPP_

#include <functional>
#include <opencv2/core.hpp>
#include <utility>

class TypeErasedProcessor
{
public:
    using ProcessorFunction = std::function<cv::Mat(const cv::Mat &)>;

    explicit TypeErasedProcessor(ProcessorFunction function)
        : function_(std::move(function)) {}

    cv::Mat process(const cv::Mat &input) const
    {
        return function_(input);
    }

private:
    ProcessorFunction function_;
};

#endif