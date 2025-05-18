#include "image_processor_package/factory.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("bridge_node");

    std::string processorType{"passthrough"};
    auto processor = ProcessorFactory::create(processorType);

    std::string sourceType{"mock"};
    auto source = SourceFactory::create(sourceType, node); // inputの部分をCameraSourceとしてbridgeさせた

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node); // これがないとspinが動かない

    // ---
    // NOTE:
    // ROSでは一般的にPublisher/Subscriberモデル（Push型）が使われる。
    // しかしBridgeパターンでは「使う側が処理の流れを制御する」Pull型の設計が必要。
    // Push型だと、データ到着のタイミングでCallbackが呼ばれるため、
    // 処理の主導権がソース側にあり、main関数が抽象の橋を架ける余地がなくなる。
    // そのため、Bridge構成を成立させるにはPull型（get_frame()）への変換が不可欠。
    // ---
    while (rclcpp::ok())
    {
        // spin_some() は登録されたコールバック（e.g. Subscription）を“非ブロッキングで1回分”実行
        // Pull型にする場合でも、callbackでの画像取得処理を実行させる必要がある
        exec.spin_some();

        // 画像フレームが来ていれば処理（pull型インターフェース）
        // subscribe型が一般的で今回bridgeにするためにあえて
        if (auto frame = source->get_frame())
        {
            cv::Mat result = processor->process(*frame);
            // 可視化（ROSらしくするならPublisher経由でrvizへ）
            cv::imshow("Bridge Processed Image", result);
            cv::waitKey(1);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }

    rclcpp::shutdown();
    return 0;
}