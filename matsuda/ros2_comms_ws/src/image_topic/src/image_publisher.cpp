#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <sensor_msgs/msg/image.hpp>
#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>
#include <thread>
#include <atomic>
#include <mutex>

using namespace std::chrono_literals;

class ImagePublisher : public rclcpp::Node
{
public:
    ImagePublisher() : Node("image_publisher"), count_(0)
    {
        // 画像トピックのパブリッシャーを作成
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image_topic", 10);
        
        // 画像パラメータを設定
        width_ = 640;
        height_ = 480;
        channels_ = 3; // BGR
        
        // 最新のフレームを保持する変数
        latest_frame_ = cv::Mat::zeros(height_, width_, CV_8UC3);
        frame_updated_ = false;
        
        // HTTPリクエスト時刻を初期化（2秒前）
        last_http_request_ = std::chrono::steady_clock::now() - std::chrono::seconds(2);
        
        RCLCPP_INFO(this->get_logger(), "画像パブリッシャーを開始しました");
        RCLCPP_INFO(this->get_logger(), "SSHで受信した画像をROS2トピックで送信します");
        
        // 画像取得スレッドを別スレッドで起動
        http_server_thread_ = std::thread(&ImagePublisher::start_http_server, this);
        
        // タイマーを作成（2Hz = 500ms間隔）
            timer_ = this->create_wall_timer(
                500ms, std::bind(&ImagePublisher::timer_callback, this));
    }

    ~ImagePublisher()
    {
        if (http_server_thread_.joinable()) {
            http_server_thread_.join();
        }
    }

private:
    void start_http_server()
    {
        try {
            // OpenCVウィンドウを初期化
            cv::namedWindow("Image Publisher", cv::WINDOW_AUTOSIZE);
            
            RCLCPP_INFO(this->get_logger(), "画像取得スレッド開始: SSHトンネル経由で画像を取得します");
            
            while (rclcpp::ok()) {
                // ローカルPCから送信された画像を取得
                cv::Mat frame = get_image_from_http();
                
                if (!frame.empty()) {
                    std::lock_guard<std::mutex> lock(frame_mutex_);
                    latest_frame_ = frame.clone();
                    frame_updated_ = true;
                    
                    // デバッグ用にフレームを表示
                    cv::imshow("Image Publisher", frame);
                    int key = cv::waitKey(1);
                    if (key == 27) { // ESCキーで終了
                        RCLCPP_INFO(this->get_logger(), "ESCキーが押されました。終了します。");
                        rclcpp::shutdown();
                        break;
                    }
                }
                
                // 500ms間隔で画像取得
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "画像取得スレッドエラー: %s", e.what());
        }
    }
    
    cv::Mat get_image_from_http()
    {
        try {
            // HTTPリクエストの頻度制限（最低1秒間隔）
            std::lock_guard<std::mutex> lock(http_request_mutex_);
            auto now = std::chrono::steady_clock::now();
            auto time_since_last = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_http_request_);
            
            if (time_since_last.count() < 1000) {
                // 1秒未満の場合はテスト画像を返す
                return generate_test_image();
            }
            
            last_http_request_ = now;
            
            // ローカルPCから送信された画像を取得
            // より安全な方法でHTTPリクエストを送信（タイムアウト短縮、リトライ制限、接続制限）
            std::string command = "curl -s --max-time 0.5 --retry 0 --retry-max-time 0 --connect-timeout 0.5 --max-filesize 1048576 http://127.0.0.1:5009/frame > /tmp/camera_frame.jpg 2>/dev/null";
            int result = system(command.c_str());
            
            if (result == 0) {
                // 画像ファイルを読み込み
                cv::Mat frame = cv::imread("/tmp/camera_frame.jpg");
                
                if (!frame.empty()) {
                    // フレームサイズを調整
                    cv::resize(frame, frame, cv::Size(width_, height_));
                    
                    // カメラモード表示
                    cv::putText(frame, "CAMERA MODE", cv::Point(10, 30), 
                               cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
                    
                    return frame;
                }
            }
            
            // ローカルPCから画像が取得できない場合はテスト画像を生成
            return generate_test_image();
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "画像取得エラー: %s", e.what());
            return cv::Mat();
        }
    }
    
    cv::Mat generate_test_image()
    {
        cv::Mat frame = cv::Mat::zeros(height_, width_, CV_8UC3);
        
        // 背景色を設定（グレー）
        frame = cv::Scalar(100, 100, 100);
        
        // 移動する円を描画
        double time_sec = this->now().seconds();
        int center_x = static_cast<int>(320 + 150 * sin(time_sec * 2));
        int center_y = static_cast<int>(240 + 80 * cos(time_sec * 2));
        cv::circle(frame, cv::Point(center_x, center_y), 60, cv::Scalar(0, 0, 255), -1);
        
        // 固定の円
        cv::circle(frame, cv::Point(100, 100), 30, cv::Scalar(0, 255, 0), -1);
        cv::circle(frame, cv::Point(540, 100), 30, cv::Scalar(255, 0, 0), -1);
        cv::circle(frame, cv::Point(100, 380), 30, cv::Scalar(255, 255, 0), -1);
        cv::circle(frame, cv::Point(540, 380), 30, cv::Scalar(255, 0, 255), -1);
        
        // 枠線
        cv::rectangle(frame, cv::Point(10, 10), cv::Point(630, 470), cv::Scalar(255, 255, 255), 5);
        
        // テストモード表示
        cv::putText(frame, "TEST MODE", cv::Point(10, 30), 
                   cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
        
        return frame;
    }
    
    void timer_callback()
    {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        
        if (frame_updated_) {
            // OpenCVのMatをROS2のImageメッセージに変換
            auto img_msg = std::make_unique<sensor_msgs::msg::Image>();
            
            img_msg->header.stamp = this->now();
            img_msg->header.frame_id = "camera_frame";
            
            img_msg->height = latest_frame_.rows;
            img_msg->width = latest_frame_.cols;
            img_msg->encoding = "bgr8";
            img_msg->is_bigendian = false;
            img_msg->step = latest_frame_.cols * latest_frame_.channels();
            
            // 画像データをコピー
            size_t data_size = latest_frame_.rows * latest_frame_.step;
            img_msg->data.resize(data_size);
            memcpy(img_msg->data.data(), latest_frame_.data, data_size);
            
            publisher_->publish(*img_msg);
            
            count_++;
            frame_updated_ = false;
            
            if (count_ % 10 == 0) {
                RCLCPP_INFO(this->get_logger(), 
                           "画像をパブリッシュしました (フレーム数: %ld, サイズ: %dx%d)", 
                           count_, img_msg->width, img_msg->height);
            }
        }
    }
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    
    // 画像関連の変数
    int width_, height_, channels_;
    size_t count_;
    
    // フレーム管理
    cv::Mat latest_frame_;
    std::mutex frame_mutex_;
    bool frame_updated_;
    std::thread http_server_thread_;
    std::chrono::steady_clock::time_point last_http_request_;
    std::mutex http_request_mutex_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImagePublisher>());
    rclcpp::shutdown();
    return 0;
}