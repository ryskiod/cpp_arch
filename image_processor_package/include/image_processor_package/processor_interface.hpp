#ifndef PROCESSOR_INTERFACE_HPP_
#define PROCESSOR_INTERFACE_HPP_

#include <opencv2/opencv.hpp>

class IProcessor
{
public:
    virtual ~IProcessor() = default;

    virtual cv::Mat process(const cv::Mat &input) = 0;
};

#endif