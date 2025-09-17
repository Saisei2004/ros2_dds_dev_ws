#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iomanip>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "builtin_interfaces/msg/time.hpp"

using namespace std::chrono_literals;

class TimePublisher : public rclcpp::Node
{
public:
  TimePublisher()
  : Node("time_publisher")
  {
    // パブリッシャーを作成（トピック名: time_topic）
    publisher_ = this->create_publisher<builtin_interfaces::msg::Time>("time_topic", 10);
    
    // タイマーを作成（1秒間隔で時間を送信）
    timer_ = this->create_wall_timer(
      1000ms, std::bind(&TimePublisher::timer_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "Time Publisher started");
  }

private:
  void timer_callback()
  {
    // 現在時刻を取得
    auto now = this->now();
    
    // メッセージを作成
    auto message = builtin_interfaces::msg::Time();
    message.sec = now.seconds();
    message.nanosec = now.nanoseconds() % 1000000000;
    
    // メッセージをパブリッシュ
    publisher_->publish(message);
    
    // 時刻を人間が読みやすい形式でログに出力
    auto time_t = std::chrono::system_clock::to_time_t(
      std::chrono::system_clock::now());
    auto tm = *std::localtime(&time_t);
    
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");
    
    RCLCPP_INFO(this->get_logger(), "Publishing time: %s (sec: %d, nanosec: %u)", 
                oss.str().c_str(), message.sec, message.nanosec);
  }
  
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TimePublisher>());
  rclcpp::shutdown();
  return 0;
}
