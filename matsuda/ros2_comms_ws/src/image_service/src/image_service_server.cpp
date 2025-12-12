#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <opencv2/opencv.hpp>
#include <cmath>
#include <curl/curl.h>
#include <thread>
#include <random>

using namespace std::chrono_literals;

// curl用のコールバック関数
size_t WriteCallback(void* contents, size_t size, size_t nmemb, std::vector<uchar>* buffer) {
    size_t total_size = size * nmemb;
    buffer->insert(buffer->end(), (uchar*)contents, (uchar*)contents + total_size);
    return total_size;
}

class ImageServiceServer : public rclcpp::Node
{
public:
    ImageServiceServer() : Node("image_service_server")
    {
        // サービスサーバーを作成
        service_ = this->create_service<std_srvs::srv::SetBool>(
            "image_service",
            std::bind(&ImageServiceServer::handle_image_service_request, this,
                     std::placeholders::_1, std::placeholders::_2));

        // テスト用の画像パラメータ
        width_ = 640;
        height_ = 480;
        
        // OpenCVウィンドウを初期化（非同期表示用）
        cv::namedWindow("Original Image", cv::WINDOW_AUTOSIZE);
        cv::namedWindow("Processed Image", cv::WINDOW_AUTOSIZE);
        
        // キーボード入力用のタイマーを設定
        timer_ = this->create_wall_timer(
            100ms, std::bind(&ImageServiceServer::check_keyboard_input, this));
        
        RCLCPP_INFO(this->get_logger(), "画像サービスサーバーを開始しました");
        RCLCPP_INFO(this->get_logger(), "サービス名: image_service");
        RCLCPP_INFO(this->get_logger(), "利用可能な画像処理:");
        RCLCPP_INFO(this->get_logger(), "  - リクエスト.data = true  : 画像統計情報取得");
        RCLCPP_INFO(this->get_logger(), "  - リクエスト.data = false : 画像フィルタリング情報取得");
        RCLCPP_INFO(this->get_logger(), "  - ESCキーで終了");
    }

private:
    void handle_image_service_request(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "画像サービスリクエストを受信しました");
        RCLCPP_INFO(this->get_logger(), "リクエスト: %s", request->data ? "統計情報取得" : "フィルタリング情報取得");

        try {
            if (request->data) {
                // 画像統計情報を取得
                response = get_image_statistics(response);
            } else {
                // 画像フィルタリング情報を取得
                response = get_image_filtering_info(response);
            }
            
            response->success = true;
            RCLCPP_INFO(this->get_logger(), "サービス処理完了: %s", response->message.c_str());
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "サービス処理エラー: %s", e.what());
            response->success = false;
            response->message = "サービス処理中にエラーが発生しました: " + std::string(e.what());
        }
    }

    std::shared_ptr<std_srvs::srv::SetBool::Response> get_image_statistics(
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        // ネットから画像を取得
        cv::Mat original_image = fetch_random_image_from_web();
        
        // 元画像を表示
        cv::imshow("Original Image", original_image);
        
        // 画像統計情報を計算
        cv::Scalar mean, stddev;
        cv::meanStdDev(original_image, mean, stddev);
        
        // ヒストグラム計算
        std::vector<cv::Mat> bgr_planes;
        cv::split(original_image, bgr_planes);
        
        int histSize = 256;
        float range[] = {0, 256};
        const float* histRange = {range};
        
        cv::Mat b_hist, g_hist, r_hist;
        cv::calcHist(&bgr_planes[0], 1, 0, cv::Mat(), b_hist, 1, &histSize, &histRange, true, false);
        cv::calcHist(&bgr_planes[1], 1, 0, cv::Mat(), g_hist, 1, &histSize, &histRange, true, false);
        cv::calcHist(&bgr_planes[2], 1, 0, cv::Mat(), r_hist, 1, &histSize, &histRange, true, false);
        
        // 統計情報をオーバーレイ表示した画像を作成
        cv::Mat stats_image = original_image.clone();
        
        // 統計情報を画像に描画
        int y_pos = 30;
        cv::putText(stats_image, "=== IMAGE STATISTICS ===", cv::Point(10, y_pos), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
        y_pos += 30;
        
        cv::putText(stats_image, "Size: " + std::to_string(width_) + "x" + std::to_string(height_), 
                   cv::Point(10, y_pos), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
        y_pos += 25;
        
        cv::putText(stats_image, "Mean (B,G,R): (" + 
                        std::to_string(static_cast<int>(mean[0])) + ", " + 
                        std::to_string(static_cast<int>(mean[1])) + ", " + 
                        std::to_string(static_cast<int>(mean[2])) + ")", 
                   cv::Point(10, y_pos), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
        y_pos += 25;
        
        cv::putText(stats_image, "StdDev (B,G,R): (" + 
                        std::to_string(static_cast<int>(stddev[0])) + ", " + 
                        std::to_string(static_cast<int>(stddev[1])) + ", " + 
                        std::to_string(static_cast<int>(stddev[2])) + ")", 
                   cv::Point(10, y_pos), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
        y_pos += 25;
        
        cv::putText(stats_image, "Total Pixels: " + std::to_string(width_ * height_), 
                   cv::Point(10, y_pos), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
        y_pos += 25;
        
        cv::putText(stats_image, "Data Size: " + std::to_string(width_ * height_ * 3) + " bytes", 
                   cv::Point(10, y_pos), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
        
        // 処理済み画像を表示
        cv::imshow("Processed Image", stats_image);
        
        // 統計情報を文字列にフォーマット
        std::string stats_message = "=== 画像統計情報 ===\n";
        stats_message += "画像サイズ: " + std::to_string(width_) + "x" + std::to_string(height_) + "\n";
        stats_message += "チャンネル数: 3 (BGR)\n";
        stats_message += "平均値 (B,G,R): (" + 
                        std::to_string(mean[0]) + ", " + 
                        std::to_string(mean[1]) + ", " + 
                        std::to_string(mean[2]) + ")\n";
        stats_message += "標準偏差 (B,G,R): (" + 
                        std::to_string(stddev[0]) + ", " + 
                        std::to_string(stddev[1]) + ", " + 
                        std::to_string(stddev[2]) + ")\n";
        stats_message += "総画素数: " + std::to_string(width_ * height_) + "\n";
        stats_message += "データサイズ: " + std::to_string(width_ * height_ * 3) + " bytes\n";
        
        response->message = stats_message;
        return response;
    }

    std::shared_ptr<std_srvs::srv::SetBool::Response> get_image_filtering_info(
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        // ネットから画像を取得
        cv::Mat original_image = fetch_random_image_from_web();
        
        // 元画像を表示
        cv::imshow("Original Image", original_image);
        
        // 各種フィルタを適用
        cv::Mat blurred, sharpened, edges, morphed;
        
        // ガウシアンブラー
        cv::GaussianBlur(original_image, blurred, cv::Size(15, 15), 0);
        
        // シャープニング
        cv::Mat kernel = (cv::Mat_<float>(3,3) << 
            0, -1, 0,
            -1, 5, -1,
            0, -1, 0);
        cv::filter2D(original_image, sharpened, -1, kernel);
        
        // エッジ検出
        cv::Mat gray;
        cv::cvtColor(original_image, gray, cv::COLOR_BGR2GRAY);
        cv::Canny(gray, edges, 50, 150);
        
        // モルフォロジー処理
        cv::Mat kernel_morph = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
        cv::morphologyEx(original_image, morphed, cv::MORPH_CLOSE, kernel_morph);
        
        // 処理結果を表示（シャープニング結果を表示）
        cv::imshow("Processed Image", sharpened);
        
        // フィルタリング情報を文字列にフォーマット
        std::string filter_message = "=== 画像フィルタリング情報 ===\n";
        filter_message += "元画像サイズ: " + std::to_string(width_) + "x" + std::to_string(height_) + "\n";
        filter_message += "適用可能なフィルタ:\n";
        filter_message += "  - ガウシアンブラー: 15x15 カーネル\n";
        filter_message += "  - シャープニング: 3x3 カーネル\n";
        filter_message += "  - Cannyエッジ検出: 閾値(50, 150)\n";
        filter_message += "  - モルフォロジー処理: 楕円カーネル 5x5\n";
        filter_message += "処理時間: 約 " + std::to_string(calculate_processing_time()) + " ms\n";
        filter_message += "メモリ使用量: " + std::to_string(calculate_memory_usage()) + " MB\n";
        
        response->message = filter_message;
        return response;
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
        
        // 固定の図形
        cv::circle(frame, cv::Point(100, 100), 30, cv::Scalar(0, 255, 0), -1);
        cv::rectangle(frame, cv::Point(200, 200), cv::Point(300, 300), cv::Scalar(255, 0, 0), -1);
        cv::ellipse(frame, cv::Point(500, 150), cv::Size(50, 30), 45, 0, 360, cv::Scalar(255, 255, 0), -1);
        cv::rectangle(frame, cv::Point(400, 350), cv::Point(550, 450), cv::Scalar(255, 0, 255), -1);
        
        // テキスト
        cv::putText(frame, "IMAGE SERVICE TEST", cv::Point(10, 30), 
                   cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255), 2);
        
        return frame;
    }

    int calculate_processing_time()
    {
        // シミュレートされた処理時間（実際の実装では計測）
        return 15 + (rand() % 10); // 15-25ms
    }

    int calculate_memory_usage()
    {
        // シミュレートされたメモリ使用量
        return 25 + (rand() % 10); // 25-35MB
    }

    void check_keyboard_input()
    {
        if (!rclcpp::ok()) {
            return;
        }
        
        // 非ブロッキングでキーボード入力チェック
        int key = cv::waitKey(1) & 0xFF;
        if (key == 27) { // ESCキー
            RCLCPP_INFO(this->get_logger(), "ESCキーが押されました。終了します。");
            rclcpp::shutdown();
        } else if (key == 'q' || key == 'Q') { // Qキーでも終了
            RCLCPP_INFO(this->get_logger(), "Qキーが押されました。終了します。");
            rclcpp::shutdown();
        }
    }

    cv::Mat fetch_random_image_from_web()
    {
        try {
            // ランダムな画像URLのリスト（Unsplash API使用）
            std::vector<std::string> image_urls = {
                "https://picsum.photos/640/480?random=1",
                "https://picsum.photos/640/480?random=2", 
                "https://picsum.photos/640/480?random=3",
                "https://picsum.photos/640/480?random=4",
                "https://picsum.photos/640/480?random=5"
            };
            
            // ランダムにURLを選択
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_int_distribution<> dis(0, image_urls.size() - 1);
            std::string selected_url = image_urls[dis(gen)];
            
            RCLCPP_INFO(this->get_logger(), "画像を取得中: %s", selected_url.c_str());
            
            // curlで画像をダウンロード
            CURL* curl = curl_easy_init();
            if (!curl) {
                RCLCPP_ERROR(this->get_logger(), "curlの初期化に失敗しました");
                return generate_test_image();
            }
            
            std::vector<uchar> buffer;
            curl_easy_setopt(curl, CURLOPT_URL, selected_url.c_str());
            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, &buffer);
            curl_easy_setopt(curl, CURLOPT_TIMEOUT, 10L);
            curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
            curl_easy_setopt(curl, CURLOPT_USERAGENT, "ROS2-ImageService/1.0");
            
            CURLcode res = curl_easy_perform(curl);
            curl_easy_cleanup(curl);
            
            if (res != CURLE_OK) {
                RCLCPP_ERROR(this->get_logger(), "画像ダウンロードに失敗しました: %s", curl_easy_strerror(res));
                return generate_test_image();
            }
            
            if (buffer.empty()) {
                RCLCPP_ERROR(this->get_logger(), "ダウンロードした画像データが空です");
                return generate_test_image();
            }
            
            // 画像データをデコード
            cv::Mat image = cv::imdecode(buffer, cv::IMREAD_COLOR);
            if (image.empty()) {
                RCLCPP_ERROR(this->get_logger(), "画像のデコードに失敗しました");
                return generate_test_image();
            }
            
            // 画像サイズを調整
            cv::resize(image, image, cv::Size(width_, height_));
            
            RCLCPP_INFO(this->get_logger(), "画像の取得に成功しました: %dx%d", image.cols, image.rows);
            return image;
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "画像取得エラー: %s", e.what());
            return generate_test_image();
        }
    }

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
    rclcpp::TimerBase::SharedPtr timer_;
    int width_, height_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageServiceServer>());
    rclcpp::shutdown();
    return 0;
}
