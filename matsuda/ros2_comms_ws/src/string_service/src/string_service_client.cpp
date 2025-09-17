#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;

class StringServiceClient : public rclcpp::Node
{
public:
  StringServiceClient()
  : Node("string_service_client")
  {
    // サービスクライアントを作成（サービス名: string_service）
    client_ = this->create_client<std_srvs::srv::SetBool>("string_service");
    
    // タイマーを作成（3秒間隔でリクエストを送信）
    timer_ = this->create_wall_timer(
      3000ms, std::bind(&StringServiceClient::timer_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "String Service Client started");
  }

private:
  void timer_callback()
  {
    // サービスが利用可能かチェック
    if (!client_->wait_for_service(1s)) {
      RCLCPP_WARN(this->get_logger(), "Service not available, waiting...");
      return;
    }
    
    // リクエストを作成（交互にtrue/falseを送信）
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = (request_count_ % 2 == 0);
    request_count_++;
    
    RCLCPP_INFO(this->get_logger(), "Sending request: data=%s", request->data ? "true" : "false");
    
    // 非同期でリクエストを送信
    auto future = client_->async_send_request(request);
    
    // レスポンスを待機（タイムアウト: 2秒）
    auto status = future.wait_for(2s);
    if (status == std::future_status::ready) {
      auto response = future.get();
      RCLCPP_INFO(this->get_logger(), "Received response: success=%s, message='%s'",
                  response->success ? "true" : "false", response->message.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Service call timed out");
    }
  }
  
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  int request_count_ = 0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StringServiceClient>());
  rclcpp::shutdown();
  return 0;
}
