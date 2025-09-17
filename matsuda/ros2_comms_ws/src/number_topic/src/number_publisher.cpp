#include <chrono>
#include <functional>
#include <memory>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

class NumberPublisher : public rclcpp::Node
{
public:
  NumberPublisher()
  : Node("number_publisher")
  {
    // パブリッシャーを作成（トピック名: number_topic）
    publisher_ = this->create_publisher<std_msgs::msg::Int32>("number_topic", 10);
    
    // タイマーを作成（1秒間隔で数値を送信）
    timer_ = this->create_wall_timer(
      1000ms, std::bind(&NumberPublisher::timer_callback, this));
    
    // 乱数生成器を初期化
    random_generator_.seed(std::chrono::steady_clock::now().time_since_epoch().count());
    distribution_ = std::uniform_int_distribution<int>(1, 100);
    
    RCLCPP_INFO(this->get_logger(), "Number Publisher started");
  }

private:
  void timer_callback()
  {
    // メッセージを作成（1-100の乱数を生成）
    auto message = std_msgs::msg::Int32();
    message.data = distribution_(random_generator_);
    
    // メッセージをパブリッシュ
    publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "Publishing number: %d", message.data);
  }
  
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
  std::mt19937 random_generator_;
  std::uniform_int_distribution<int> distribution_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NumberPublisher>());
  rclcpp::shutdown();
  return 0;
}
