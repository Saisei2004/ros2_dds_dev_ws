#include <memory>
#include <string>
#include <vector>
#include <sensor_msgs/msg/image.hpp>
#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>
#include <thread>
#include <atomic>
#include <mutex>

class ImageSubscriber : public rclcpp::Node
{
public:
    ImageSubscriber() : Node("image_subscriber"), frame_count_(0)
    {
        // 画像トピックのサブスクライバーを作成
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "image_topic", 10, 
            std::bind(&ImageSubscriber::topic_callback, this, std::placeholders::_1));
            
        RCLCPP_INFO(this->get_logger(), "画像サブスクライバーを開始しました");
        RCLCPP_INFO(this->get_logger(), "トピック: image_topic");
        RCLCPP_INFO(this->get_logger(), "ROS2トピックから受信した画像をSSHでローカルに送信します");
        RCLCPP_INFO(this->get_logger(), "ポート5010でHTTPサーバーを起動中...");
        
        // HTTPサーバーを別スレッドで起動
        http_server_thread_ = std::thread(&ImageSubscriber::start_http_server, this);
        
        // 統計情報表示用のタイマー（5秒間隔）
        stats_timer_ = this->create_wall_timer(
            std::chrono::seconds(5), 
            std::bind(&ImageSubscriber::print_stats, this));
        
        start_time_ = this->now();
    }

    ~ImageSubscriber()
    {
        if (http_server_thread_.joinable()) {
            http_server_thread_.join();
        }
    }

private:
    void start_http_server()
    {
        try {
            cv::namedWindow("Image Subscriber", cv::WINDOW_AUTOSIZE);
            
            RCLCPP_INFO(this->get_logger(), "HTTPサーバー開始: http://127.0.0.1:5010/video");
            RCLCPP_INFO(this->get_logger(), "ESCキーまたはCtrl+Cで終了できます");
            
            while (rclcpp::ok()) {
                std::lock_guard<std::mutex> lock(frame_mutex_);
                
                if (!latest_frame_.empty()) {
                    // フレームを表示
                    cv::imshow("Image Subscriber", latest_frame_);
                    int key = cv::waitKey(1);
                    if (key == 27) { // ESCキーで終了
                        RCLCPP_INFO(this->get_logger(), "ESCキーが押されました。終了します。");
                        rclcpp::shutdown();
                        break;
                    }
                }
                
                std::this_thread::sleep_for(std::chrono::milliseconds(33)); // 30FPS
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "HTTPサーバーエラー: %s", e.what());
        }
    }
    
    void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            frame_count_++;
            last_frame_time_ = this->now();
            
            if (frame_count_ <= 3) {
                RCLCPP_INFO(this->get_logger(), 
                           "画像受信 #%ld: %dx%d, エンコーディング: %s, データサイズ: %zu bytes", 
                           frame_count_, msg->width, msg->height, 
                           msg->encoding.c_str(), msg->data.size());
            }
            
            // ROS2のImageメッセージをOpenCVのMatに変換
            cv::Mat frame = ros2_to_opencv(msg);
            
            if (!frame.empty()) {
                std::lock_guard<std::mutex> lock(frame_mutex_);
                latest_frame_ = frame.clone();
                
                // 統計情報をフレームに描画
                draw_stats_on_frame(latest_frame_);
            }
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "画像処理エラー: %s", e.what());
        }
    }
    
    cv::Mat ros2_to_opencv(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            cv::Mat frame;
            
            if (msg->encoding == "bgr8") {
                frame = cv::Mat(msg->height, msg->width, CV_8UC3, const_cast<uint8_t*>(msg->data.data()));
                frame = frame.clone(); // データをコピー
            } else if (msg->encoding == "rgb8") {
                frame = cv::Mat(msg->height, msg->width, CV_8UC3, const_cast<uint8_t*>(msg->data.data()));
                cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);
            } else if (msg->encoding == "mono8") {
                frame = cv::Mat(msg->height, msg->width, CV_8UC1, const_cast<uint8_t*>(msg->data.data()));
                cv::cvtColor(frame, frame, cv::COLOR_GRAY2BGR);
            } else {
                RCLCPP_WARN(this->get_logger(), "未対応のエンコーディング: %s", msg->encoding.c_str());
                return cv::Mat();
            }
            
            return frame;
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "ROS2→OpenCV変換エラー: %s", e.what());
            return cv::Mat();
        }
    }
    
    void draw_stats_on_frame(cv::Mat& frame)
    {
        try {
            // FPS計算
            auto elapsed = this->now() - start_time_;
            double fps = frame_count_ / elapsed.seconds();
            
            // 統計情報テキスト
            std::string stats_text = "ROS2 FPS: " + std::to_string(fps).substr(0, 4) + 
                                   " | Frames: " + std::to_string(frame_count_);
            
            // テキストを描画
            cv::putText(frame, stats_text, cv::Point(10, 30), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
            
            // 接続状態を表示
            cv::putText(frame, "SSH Tunnel: Connected", cv::Point(10, 60), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
            
            // 操作説明
            cv::putText(frame, "ESC: Exit", cv::Point(10, frame.rows - 20), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "統計情報描画エラー: %s", e.what());
        }
    }
    
    void print_stats()
    {
        if (frame_count_ == 0) {
            RCLCPP_INFO(this->get_logger(), "まだ画像を受信していません");
            return;
        }
        
        auto elapsed = this->now() - start_time_;
        double fps = frame_count_ / elapsed.seconds();
        
        RCLCPP_INFO(this->get_logger(), "=== ROS2画像受信統計 ===");
        RCLCPP_INFO(this->get_logger(), "受信フレーム数: %ld", frame_count_);
        RCLCPP_INFO(this->get_logger(), "経過時間: %.2f秒", elapsed.seconds());
        RCLCPP_INFO(this->get_logger(), "平均FPS: %.2f", fps);
        
        if (last_frame_time_.nanoseconds() > 0) {
            auto time_since_last = this->now() - last_frame_time_;
            RCLCPP_INFO(this->get_logger(), "最後のフレームから: %.3f秒前", time_since_last.seconds());
        }
        
        RCLCPP_INFO(this->get_logger(), "トピック: image_topic");
        RCLCPP_INFO(this->get_logger(), "HTTPサーバー: http://127.0.0.1:5010/video");
        RCLCPP_INFO(this->get_logger(), "========================");
    }
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr stats_timer_;
    std::thread http_server_thread_;
    
    // フレーム管理
    cv::Mat latest_frame_;
    std::mutex frame_mutex_;
    
    // 統計情報
    size_t frame_count_;
    rclcpp::Time start_time_;
    rclcpp::Time last_frame_time_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    RCLCPP_INFO(rclcpp::get_logger("main"), "ROS2画像サブスクライバーを起動します");
    
    rclcpp::spin(std::make_shared<ImageSubscriber>());
    rclcpp::shutdown();
    return 0;
}