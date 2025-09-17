#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <numeric>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class NumberServiceServer : public rclcpp::Node
{
public:
  NumberServiceServer()
  : Node("number_service_server")
  {
    // サービスサーバーを作成（サービス名: number_service）
    service_ = this->create_service<std_srvs::srv::SetBool>(
      "number_service",
      std::bind(&NumberServiceServer::service_callback, this, _1, _2));
    
    // 数値リストを初期化（サンプルデータ）
    numbers_ = {10, 25, 35, 42, 58, 67, 73, 81, 92, 15};
    
    RCLCPP_INFO(this->get_logger(), "Number Service Server started, waiting for requests...");
    RCLCPP_INFO(this->get_logger(), "Available numbers: [%s]", 
                vector_to_string(numbers_).c_str());
  }

private:
  void service_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    // リクエスト内容をログに出力
    RCLCPP_INFO(this->get_logger(), "Received request: data=%s", request->data ? "true" : "false");
    
    // 数値処理を実行
    std::string result_message;
    bool success = false;
    
    if (request->data) {
      // trueの場合: 数値の統計情報を計算
      result_message = calculate_statistics();
      success = true;
    } else {
      // falseの場合: 数値リストをソート
      result_message = sort_numbers();
      success = true;
    }
    
    response->success = success;
    response->message = result_message;
    
    RCLCPP_INFO(this->get_logger(), "Sending response: success=%s, message='%s'",
                response->success ? "true" : "false", response->message.c_str());
  }
  
  std::string calculate_statistics()
  {
    if (numbers_.empty()) {
      return "No numbers available for statistics";
    }
    
    int sum = std::accumulate(numbers_.begin(), numbers_.end(), 0);
    double average = static_cast<double>(sum) / numbers_.size();
    int min_val = *std::min_element(numbers_.begin(), numbers_.end());
    int max_val = *std::max_element(numbers_.begin(), numbers_.end());
    
    std::string stats = "Statistics - Count: " + std::to_string(numbers_.size()) +
                       ", Sum: " + std::to_string(sum) +
                       ", Average: " + std::to_string(average) +
                       ", Min: " + std::to_string(min_val) +
                       ", Max: " + std::to_string(max_val);
    
    return stats;
  }
  
  std::string sort_numbers()
  {
    std::vector<int> sorted_numbers = numbers_;
    std::sort(sorted_numbers.begin(), sorted_numbers.end());
    
    std::string result = "Sorted numbers: [" + vector_to_string(sorted_numbers) + "]";
    return result;
  }
  
  std::string vector_to_string(const std::vector<int>& vec)
  {
    std::string result;
    for (size_t i = 0; i < vec.size(); ++i) {
      if (i > 0) result += ", ";
      result += std::to_string(vec[i]);
    }
    return result;
  }
  
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
  std::vector<int> numbers_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NumberServiceServer>());
  rclcpp::shutdown();
  return 0;
}
