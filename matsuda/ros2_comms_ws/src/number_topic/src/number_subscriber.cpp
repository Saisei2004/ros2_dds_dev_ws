#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

using std::placeholders::_1;

class NumberSubscriber : public rclcpp::Node
{
public:
  NumberSubscriber()
  : Node("number_subscriber")
  {
    // サブスクライバーを作成（トピック名: number_topic）
    subscription_ = this->create_subscription<std_msgs::msg::Int32>(
      "number_topic", 10, std::bind(&NumberSubscriber::topic_callback, this, _1));
    
    // 統計情報の初期化
    message_count_ = 0;
    sum_ = 0;
    min_value_ = std::numeric_limits<int>::max();
    max_value_ = std::numeric_limits<int>::min();
    
    RCLCPP_INFO(this->get_logger(), "Number Subscriber started, waiting for messages...");
  }

private:
  void topic_callback(const std_msgs::msg::Int32::SharedPtr msg) const
  {
    // 統計情報を更新
    message_count_++;
    sum_ += msg->data;
    
    if (msg->data < min_value_) {
      min_value_ = msg->data;
    }
    if (msg->data > max_value_) {
      max_value_ = msg->data;
    }
    
    // 受信した数値をログに出力
    RCLCPP_INFO(this->get_logger(), "Received number: %d", msg->data);
    
    // 10回ごとに統計情報を表示
    if (message_count_ % 10 == 0) {
      double average = static_cast<double>(sum_) / message_count_;
      RCLCPP_INFO(this->get_logger(), 
                  "Statistics - Count: %d, Sum: %d, Average: %.2f, Min: %d, Max: %d",
                  message_count_, sum_, average, min_value_, max_value_);
    }
  }
  
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
  mutable int message_count_;
  mutable int sum_;
  mutable int min_value_;
  mutable int max_value_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NumberSubscriber>());
  rclcpp::shutdown();
  return 0;
}
