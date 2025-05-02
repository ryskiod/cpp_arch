#include "processor_interface.hpp"

using namespace std;

class PassthroughProcessor : public IProcessor
{
public:
    cv::Mat process(const cv::Mat &input) override
    {
        // 設計が目的なので具体的なアルゴリズム書いていない
        cout << "Called passthrough" << endl;
        return input;
    }

    ~PassthroughProcessor() final = default;
};