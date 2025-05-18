#ifndef PROCESSOR_FACTORY_HPP_
#define PROCESSOR_FACTORY_HPP_

class ProcessorFactory
{
public:
    static std::function<cv::Mat(const cv::Mat &)> create(const std::string &type)
    {
        if (type == "passthrough")
        {
            return [](const cv::Mat &img) -> cv::Mat
            {
                return img;
            };
        }
        if (type == "canny")
        {
            return [](const cv::Mat &inputImg)
            {
                cv::Mat outputImg;
                cv::Canny(inputImg, outputImg, 100, 200);
                return outputImg;
            };
        }
        throw std::invalid_argument("unknown processor type: " + type);
    }
};

#endif