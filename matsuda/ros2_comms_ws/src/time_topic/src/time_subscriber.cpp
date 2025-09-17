#include <memory>
#include <string>
#include <chrono>
#include <iomanip>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "builtin_interfaces/msg/time.hpp"

using std::placeholders::_1;

class TimeSubscriber : public rclcpp::Node
{
public:
  TimeSubscriber()
  : Node("time_subscriber")
  {
    // サブスクライバーを作成（トピック名: time_topic）
    subscription_ = this->create_subscription<builtin_interfaces::msg::Time>(
      "time_topic", 10, std::bind(&TimeSubscriber::topic_callback, this, _1));
    
    // 統計情報の初期化
    message_count_ = 0;
    first_message_time_ = rclcpp::Time(0);
    last_message_time_ = rclcpp::Time(0);
    
    RCLCPP_INFO(this->get_logger(), "Time Subscriber started, waiting for messages...");
  }

private:
  void topic_callback(const builtin_interfaces::msg::Time::SharedPtr msg) const
  {
    // メッセージ時刻をrclcpp::Timeオブジェクトに変換
    rclcpp::Time msg_time(*msg);
    
    // 統計情報を更新
    message_count_++;
    
    if (first_message_time_.seconds() == 0) {
      first_message_time_ = msg_time;
    }
    last_message_time_ = msg_time;
    
    // 時刻を人間が読みやすい形式に変換
    std::string formatted_time = format_time(msg_time);
    
    // 受信した時刻をログに出力
    RCLCPP_INFO(this->get_logger(), "Received time: %s (sec: %d, nanosec: %u)", 
                formatted_time.c_str(), msg->sec, msg->nanosec);
    
    // 10回ごとに統計情報を表示
    if (message_count_ % 10 == 0) {
      display_statistics();
    }
  }
  
  std::string format_time(const rclcpp::Time& time) const
  {
    // ROS2の時刻をstd::chronoに変換
    auto duration = std::chrono::nanoseconds(time.nanoseconds());
    auto time_point = std::chrono::system_clock::time_point(duration);
    auto time_t = std::chrono::system_clock::to_time_t(time_point);
    auto tm = *std::localtime(&time_t);
    
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");
    return oss.str();
  }
  
  void display_statistics() const
  {
    if (message_count_ > 1) {
      double time_span = (last_message_time_ - first_message_time_).seconds();
      double frequency = (message_count_ - 1) / time_span;
      
      RCLCPP_INFO(this->get_logger(), 
                  "Statistics - Messages: %d, Time span: %.2f sec, Frequency: %.2f Hz",
                  message_count_, time_span, frequency);
    }
  }
  
  rclcpp::Subscription<builtin_interfaces::msg::Time>::SharedPtr subscription_;
  mutable int message_count_;
  mutable rclcpp::Time first_message_time_;
  mutable rclcpp::Time last_message_time_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TimeSubscriber>());
  rclcpp::shutdown();
  return 0;
}
