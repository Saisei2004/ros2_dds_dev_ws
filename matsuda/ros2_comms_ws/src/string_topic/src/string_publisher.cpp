#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class StringPublisher : public rclcpp::Node
{
public:
  StringPublisher()
  : Node("string_publisher")
  {
    // パブリッシャーを作成（トピック名: string_topic）
    publisher_ = this->create_publisher<std_msgs::msg::String>("string_topic", 10);
    
    // タイマーを作成（1秒間隔でメッセージを送信）
    timer_ = this->create_wall_timer(
      1000ms, std::bind(&StringPublisher::timer_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "String Publisher started");
  }

private:
  void timer_callback()
  {
    // メッセージを作成
    auto message = std_msgs::msg::String();
    message.data = "Hello from string_publisher! Time: " + 
                   std::to_string(this->now().seconds());
    
    // メッセージをパブリッシュ
    publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  }
  
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StringPublisher>());
  rclcpp::shutdown();
  return 0;
}
