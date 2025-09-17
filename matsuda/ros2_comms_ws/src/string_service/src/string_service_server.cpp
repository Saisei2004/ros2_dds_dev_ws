#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class StringServiceServer : public rclcpp::Node
{
public:
  StringServiceServer()
  : Node("string_service_server")
  {
    // サービスサーバーを作成（サービス名: string_service）
    service_ = this->create_service<std_srvs::srv::SetBool>(
      "string_service",
      std::bind(&StringServiceServer::service_callback, this, _1, _2));
    
    RCLCPP_INFO(this->get_logger(), "String Service Server started, waiting for requests...");
  }

private:
  void service_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    // リクエスト内容をログに出力
    RCLCPP_INFO(this->get_logger(), "Received request: data=%s", request->data ? "true" : "false");
    
    // レスポンスを作成（リクエストを大文字に変換して返す）
    std::string request_str = request->data ? "TRUE" : "FALSE";
    response->success = true;
    response->message = "Server processed: " + request_str + " at time: " + 
                       std::to_string(this->now().seconds());
    
    RCLCPP_INFO(this->get_logger(), "Sending response: success=%s, message='%s'",
                response->success ? "true" : "false", response->message.c_str());
  }
  
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StringServiceServer>());
  rclcpp::shutdown();
  return 0;
}
