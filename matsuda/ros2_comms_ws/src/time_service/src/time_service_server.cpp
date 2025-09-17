#include <memory>
#include <string>
#include <chrono>
#include <iomanip>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class TimeServiceServer : public rclcpp::Node
{
public:
  TimeServiceServer()
  : Node("time_service_server")
  {
    // サービスサーバーを作成（サービス名: time_service）
    service_ = this->create_service<std_srvs::srv::SetBool>(
      "time_service",
      std::bind(&TimeServiceServer::service_callback, this, _1, _2));
    
    RCLCPP_INFO(this->get_logger(), "Time Service Server started, waiting for requests...");
  }

private:
  void service_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    // リクエスト内容をログに出力
    RCLCPP_INFO(this->get_logger(), "Received request: data=%s", request->data ? "true" : "false");
    
    // 現在時刻を取得
    auto now = this->now();
    
    std::string result_message;
    bool success = false;
    
    if (request->data) {
      // trueの場合: 詳細な時刻情報を提供
      result_message = get_detailed_time_info(now);
      success = true;
    } else {
      // falseの場合: 現在時刻を人間が読みやすい形式で提供
      result_message = get_formatted_time(now);
      success = true;
    }
    
    response->success = success;
    response->message = result_message;
    
    RCLCPP_INFO(this->get_logger(), "Sending response: success=%s, message='%s'",
                response->success ? "true" : "false", response->message.c_str());
  }
  
  std::string get_detailed_time_info(const rclcpp::Time& time)
  {
    // ROS2の時刻をstd::chronoに変換
    auto duration = std::chrono::nanoseconds(time.nanoseconds());
    auto time_point = std::chrono::system_clock::time_point(duration);
    auto time_t = std::chrono::system_clock::to_time_t(time_point);
    auto tm = *std::localtime(&time_t);
    
    // 詳細な時刻情報を構築
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");
    
    std::string detailed_info = "Detailed time info - ";
    detailed_info += "Formatted: " + oss.str() + ", ";
    detailed_info += "Unix timestamp: " + std::to_string(time.seconds()) + ", ";
    detailed_info += "Nanoseconds: " + std::to_string(time.nanoseconds() % 1000000000) + ", ";
    detailed_info += "Total nanoseconds: " + std::to_string(time.nanoseconds());
    
    return detailed_info;
  }
  
  std::string get_formatted_time(const rclcpp::Time& time)
  {
    // ROS2の時刻をstd::chronoに変換
    auto duration = std::chrono::nanoseconds(time.nanoseconds());
    auto time_point = std::chrono::system_clock::time_point(duration);
    auto time_t = std::chrono::system_clock::to_time_t(time_point);
    auto tm = *std::localtime(&time_t);
    
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");
    
    return "Current time: " + oss.str();
  }
  
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TimeServiceServer>());
  rclcpp::shutdown();
  return 0;
}
