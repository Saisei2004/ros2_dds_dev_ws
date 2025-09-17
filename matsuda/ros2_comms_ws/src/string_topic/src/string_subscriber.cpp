#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class StringSubscriber : public rclcpp::Node
{
public:
  StringSubscriber()
  : Node("string_subscriber")
  {
    // サブスクライバーを作成（トピック名: string_topic）
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "string_topic", 10, std::bind(&StringSubscriber::topic_callback, this, _1));
    
    RCLCPP_INFO(this->get_logger(), "String Subscriber started, waiting for messages...");
  }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    // 受信したメッセージをログに出力
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  }
  
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StringSubscriber>());
  rclcpp::shutdown();
  return 0;
}
