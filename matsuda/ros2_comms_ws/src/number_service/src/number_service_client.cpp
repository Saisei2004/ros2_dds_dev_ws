#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;

class NumberServiceClient : public rclcpp::Node
{
public:
  NumberServiceClient()
  : Node("number_service_client")
  {
    // サービスクライアントを作成（サービス名: number_service）
    client_ = this->create_client<std_srvs::srv::SetBool>("number_service");
    
    // タイマーを作成（5秒間隔でリクエストを送信）
    timer_ = this->create_wall_timer(
      5000ms, std::bind(&NumberServiceClient::timer_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "Number Service Client started");
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
    
    std::string request_type = request->data ? "statistics" : "sorting";
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
    
    // 10回のリクエスト後に終了
    if (request_count_ >= 10) {
      RCLCPP_INFO(this->get_logger(), "Completed 10 requests, shutting down...");
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
  rclcpp::spin(std::make_shared<NumberServiceClient>());
  rclcpp::shutdown();
  return 0;
}
