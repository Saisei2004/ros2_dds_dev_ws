#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;

class ImageServiceClient : public rclcpp::Node
{
public:
    ImageServiceClient() : Node("image_service_client")
    {
        // サービスクライアントを作成
        client_ = this->create_client<std_srvs::srv::SetBool>("image_service");

        RCLCPP_INFO(this->get_logger(), "画像サービスクライアントを開始しました");
        RCLCPP_INFO(this->get_logger(), "サービス名: image_service");
        RCLCPP_INFO(this->get_logger(), "使用方法:");
        RCLCPP_INFO(this->get_logger(), "  - 引数なし: 画像統計情報を取得");
        RCLCPP_INFO(this->get_logger(), "  - 引数 'filter': 画像フィルタリング情報を取得");
        RCLCPP_INFO(this->get_logger(), "  - 引数 'both': 両方の情報を取得");
    }

    void send_request(const std::string& request_type = "stats")
    {
        // サービスサーバーの準備を待つ
        while (!client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "サービス待機中に中断されました");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "サービスサーバーを待機中...");
        }

        if (request_type == "stats" || request_type == "both") {
            send_statistics_request();
        }
        
        if (request_type == "filter" || request_type == "both") {
            send_filtering_request();
        }
    }

private:
    void send_statistics_request()
    {
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = true; // 統計情報取得

        RCLCPP_INFO(this->get_logger(), "画像統計情報リクエストを送信中...");

        auto result = client_->async_send_request(
            request,
            std::bind(&ImageServiceClient::handle_statistics_response, this,
                     std::placeholders::_1));
    }

    void send_filtering_request()
    {
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = false; // フィルタリング情報取得

        RCLCPP_INFO(this->get_logger(), "画像フィルタリング情報リクエストを送信中...");

        auto result = client_->async_send_request(
            request,
            std::bind(&ImageServiceClient::handle_filtering_response, this,
                     std::placeholders::_1));
    }

    void handle_statistics_response(rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future)
    {
        try {
            auto response = future.get();
            
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "=== 画像統計情報レスポンス ===");
                RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
                RCLCPP_INFO(this->get_logger(), "===============================");
            } else {
                RCLCPP_ERROR(this->get_logger(), "統計情報取得に失敗しました: %s", 
                           response->message.c_str());
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "統計情報リクエストエラー: %s", e.what());
        }
    }

    void handle_filtering_response(rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future)
    {
        try {
            auto response = future.get();
            
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "=== 画像フィルタリング情報レスポンス ===");
                RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
                RCLCPP_INFO(this->get_logger(), "=====================================");
            } else {
                RCLCPP_ERROR(this->get_logger(), "フィルタリング情報取得に失敗しました: %s", 
                           response->message.c_str());
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "フィルタリング情報リクエストエラー: %s", e.what());
        }
    }

    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto client_node = std::make_shared<ImageServiceClient>();

    // コマンドライン引数に基づいてリクエストタイプを決定
    std::string request_type = "stats"; // デフォルト
    if (argc > 1) {
        request_type = argv[1];
    }

    // リクエストを送信
    client_node->send_request(request_type);

    // レスポンスを待つために少し待機
    std::this_thread::sleep_for(5s);

    rclcpp::shutdown();
    return 0;
}
