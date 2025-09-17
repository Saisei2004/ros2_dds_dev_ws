#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;

class TimeServiceClient : public rclcpp::Node
{
public:
  TimeServiceClient()
  : Node("time_service_client")
  {
    // サービスクライアントを作成（サービス名: time_service）
    client_ = this->create_client<std_srvs::srv::SetBool>("time_service");
    
    // タイマーを作成（3秒間隔でリクエストを送信）
    timer_ = this->create_wall_timer(
      3000ms, std::bind(&TimeServiceClient::timer_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "Time Service Client started");
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
    
    std::string request_type = request->data ? "detailed time info" : "formatted time";
    RCLCPP_INFO(this->get_logger(), "Sending request: data=%s (%s)", 
                request->data ? "true" : "false", request_type.c_str());
    
    // 非同期でリクエストを送信
    auto future = client_->async_send_request(request);
    
    // レスポンスを待機（タイムアウト: 3秒）
    auto status = future.wait_for(3s);
    if (status == std::future_status::ready) {
      auto response = future.get();
      if (response->success) {
        RCLCPP_INFO(this->get_logger(), "Received response: %s", response->message.c_str());
      } else {
        RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", response->message.c_str());
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "Service call timed out");
    }
    
    // 8回のリクエスト後に終了
    if (request_count_ >= 8) {
      RCLCPP_INFO(this->get_logger(), "Completed 8 requests, shutting down...");
      rclcpp::shutdown();
    }
  }
  
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  int request_count_ = 0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TimeServiceClient>());
  rclcpp::shutdown();
  return 0;
}
