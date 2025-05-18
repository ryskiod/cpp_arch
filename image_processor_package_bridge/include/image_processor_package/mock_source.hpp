#ifndef MOCK_SOURCE_HPP_
#define MOCK_SOURCE_HPP_

#include "img_source_interface.hpp"

class MockSource : public ImgSource
{
public:
    std::optional<cv::Mat> get_frame() override;

private:
    bool already_sent_ = false;
};

#endif