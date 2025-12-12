#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/image.hpp"

// Audio libraries
#include <alsa/asoundlib.h>
#include <portaudio.h>
#include <sndfile.h>

// Visual feedback
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

class AudioSubscriber : public rclcpp::Node
{
public:
    AudioSubscriber() : Node("audio_subscriber"),
                        is_playing_(false),
                        test_mode_(false),
                        audio_buffer_(),
                        playback_thread_(),
                        cv_windows_available_(false),
                        simulate_playback_(false),
                        playback_frequency_(440.0)
    {
        // Create subscriber for audio data
        audio_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "audio_topic", 10, std::bind(&AudioSubscriber::audio_callback, this, std::placeholders::_1));
        
        // Create subscriber for audio visualization
        visualization_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "audio_visualization", 10, std::bind(&AudioSubscriber::visualization_callback, this, std::placeholders::_1));

        // Initialize audio output system
        if (!initialize_audio_output()) {
            RCLCPP_WARN(this->get_logger(), "Audio output initialization failed - running in test mode");
            test_mode_ = true;
        } else {
            test_mode_ = false;
        }

        // Initialize OpenCV windows (only if not in headless mode)
        try {
            cv::namedWindow("Audio Visualization", cv::WINDOW_AUTOSIZE);
            cv::namedWindow("Audio Control Panel", cv::WINDOW_AUTOSIZE);
            cv_windows_available_ = true;
        } catch (const cv::Exception& e) {
            RCLCPP_WARN(this->get_logger(), "OpenCV windows not available: %s", e.what());
            cv_windows_available_ = false;
        }

        // Start playback thread
        playback_thread_ = std::thread(&AudioSubscriber::playback_loop, this);

        // Create timer for UI updates
        ui_timer_ = this->create_wall_timer(
            50ms, std::bind(&AudioSubscriber::update_ui, this));

        // Start keyboard input thread for playback simulation
        keyboard_thread_ = std::thread(&AudioSubscriber::keyboard_input_loop, this);

        RCLCPP_INFO(this->get_logger(), "Audio Subscriber started");
        RCLCPP_INFO(this->get_logger(), "Subscribing to topic: audio_topic");
        RCLCPP_INFO(this->get_logger(), "Audio output enabled");
        RCLCPP_INFO(this->get_logger(), "Visual feedback enabled");
        RCLCPP_INFO(this->get_logger(), "Controls: 'p' to toggle playback, 't' to simulate playback, '1-9' to change frequency, 'q' to quit");
    }

    ~AudioSubscriber()
    {
        is_playing_ = false;
        if (playback_thread_.joinable()) {
            playback_thread_.join();
        }
        if (keyboard_thread_.joinable()) {
            keyboard_thread_.join();
        }
        cleanup_audio();
        if (cv_windows_available_) {
            try {
                cv::destroyAllWindows();
            } catch (const cv::Exception& e) {
                RCLCPP_WARN(this->get_logger(), "Failed to destroy OpenCV windows: %s", e.what());
            }
        }
    }

private:
    // Audio parameters
    static constexpr int SAMPLE_RATE = 44100;
    static constexpr int CHANNELS = 1;
    static constexpr int BUFFER_SIZE = 1024;
    
    // Audio system
    snd_pcm_t* pcm_output_handle_;
    PaStream* portaudio_output_stream_;
    std::vector<float> audio_buffer_;
    std::mutex buffer_mutex_;
    std::atomic<bool> is_playing_;
    std::atomic<bool> test_mode_;
    std::thread playback_thread_;
    
    // Visual feedback
    cv::Mat current_visualization_;
    std::mutex vis_mutex_;
    bool has_new_visualization_;
    bool cv_windows_available_;
    
    // Audio statistics
    double current_rms_;
    double current_peak_;
    std::vector<double> current_frequency_data_;
    
    // Audio simulation
    std::atomic<bool> simulate_playback_;
    std::atomic<double> playback_frequency_;
    std::thread keyboard_thread_;
    
    // ROS2 subscribers
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr audio_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr visualization_subscription_;
    rclcpp::TimerBase::SharedPtr ui_timer_;

    void audio_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.empty()) {
            return;
        }

        // Extract audio statistics
        current_rms_ = msg->data[0];
        current_peak_ = msg->data.size() > 1 ? msg->data[1] : 0.0;
        
        // Extract frequency data if available
        if (msg->data.size() > 2) {
            current_frequency_data_.clear();
            current_frequency_data_.insert(current_frequency_data_.end(), 
                                         msg->data.begin() + 2, msg->data.end());
        }

        // Generate audio output based on input
        generate_audio_output();

        RCLCPP_DEBUG(this->get_logger(), "Received audio data - RMS: %.4f, Peak: %.4f", 
                    current_rms_, current_peak_);
    }

    void visualization_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Convert ROS2 Image to OpenCV Mat
        cv::Mat image(msg->height, msg->width, CV_8UC3, const_cast<uchar*>(msg->data.data()));
        
        std::lock_guard<std::mutex> lock(vis_mutex_);
        current_visualization_ = image.clone();
        has_new_visualization_ = true;
    }

    bool initialize_audio_output()
    {
        // Try PortAudio first
        PaError err = Pa_Initialize();
        if (err != paNoError) {
            RCLCPP_ERROR(this->get_logger(), "PortAudio initialization failed: %s", Pa_GetErrorText(err));
            return initialize_alsa_output();
        }

        PaStreamParameters output_parameters;
        output_parameters.device = Pa_GetDefaultOutputDevice();
        output_parameters.channelCount = CHANNELS;
        output_parameters.sampleFormat = paFloat32;
        output_parameters.suggestedLatency = Pa_GetDeviceInfo(output_parameters.device)->defaultLowOutputLatency;
        output_parameters.hostApiSpecificStreamInfo = nullptr;

        err = Pa_OpenStream(&portaudio_output_stream_, nullptr, &output_parameters,
                           SAMPLE_RATE, paFramesPerBufferUnspecified,
                           paClipOff, nullptr, nullptr);

        if (err != paNoError) {
            RCLCPP_ERROR(this->get_logger(), "PortAudio output stream open failed: %s", Pa_GetErrorText(err));
            Pa_Terminate();
            return initialize_alsa_output();
        }

        err = Pa_StartStream(portaudio_output_stream_);
        if (err != paNoError) {
            RCLCPP_ERROR(this->get_logger(), "PortAudio output stream start failed: %s", Pa_GetErrorText(err));
            Pa_CloseStream(portaudio_output_stream_);
            Pa_Terminate();
            return initialize_alsa_output();
        }

        RCLCPP_INFO(this->get_logger(), "PortAudio output initialized successfully");
        return true;
    }

    bool initialize_alsa_output()
    {
        int err = snd_pcm_open(&pcm_output_handle_, "default", SND_PCM_STREAM_PLAYBACK, 0);
        if (err < 0) {
            RCLCPP_ERROR(this->get_logger(), "ALSA PCM output open failed: %s", snd_strerror(err));
            return false;
        }

        snd_pcm_hw_params_t* hw_params;
        snd_pcm_hw_params_alloca(&hw_params);
        snd_pcm_hw_params_any(pcm_output_handle_, hw_params);
        snd_pcm_hw_params_set_access(pcm_output_handle_, hw_params, SND_PCM_ACCESS_RW_INTERLEAVED);
        snd_pcm_hw_params_set_format(pcm_output_handle_, hw_params, SND_PCM_FORMAT_FLOAT);
        unsigned int sample_rate = SAMPLE_RATE;
        snd_pcm_hw_params_set_rate_near(pcm_output_handle_, hw_params, &sample_rate, 0);
        snd_pcm_hw_params_set_channels(pcm_output_handle_, hw_params, CHANNELS);

        err = snd_pcm_hw_params(pcm_output_handle_, hw_params);
        if (err < 0) {
            RCLCPP_ERROR(this->get_logger(), "ALSA output hw_params set failed: %s", snd_strerror(err));
            snd_pcm_close(pcm_output_handle_);
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "ALSA output initialized successfully");
        return true;
    }

    void generate_audio_output()
    {
        // Generate audio based on input characteristics
        std::vector<float> output_buffer(BUFFER_SIZE);
        
        for (size_t i = 0; i < BUFFER_SIZE; ++i) {
            float sample = 0.0f;
            
            // Generate tone based on RMS level (frequency modulation)
            double frequency = 440.0 + (current_rms_ * 200.0); // 440Hz base + RMS modulation
            double phase = 2.0 * M_PI * frequency * i / SAMPLE_RATE;
            
            // Generate waveform based on peak level (amplitude modulation)
            double amplitude = current_peak_ * 0.5; // Scale down for safety
            
            // Create sine wave with envelope
            sample = amplitude * std::sin(phase);
            
            // Add harmonics based on frequency data
            if (!current_frequency_data_.empty()) {
                for (size_t j = 0; j < std::min(current_frequency_data_.size(), size_t(10)); ++j) {
                    double harmonic_freq = frequency * (j + 2);
                    double harmonic_phase = 2.0 * M_PI * harmonic_freq * i / SAMPLE_RATE;
                    double harmonic_amp = current_frequency_data_[j] * 0.1;
                    sample += harmonic_amp * std::sin(harmonic_phase);
                }
            }
            
            output_buffer[i] = sample;
        }

        // Add to playback buffer
        {
            std::lock_guard<std::mutex> lock(buffer_mutex_);
            audio_buffer_.insert(audio_buffer_.end(), output_buffer.begin(), output_buffer.end());
        }
    }

    void playback_loop()
    {
        std::vector<float> playback_buffer(BUFFER_SIZE);
        
        while (rclcpp::ok()) {
            // Get audio data from buffer
            {
                std::lock_guard<std::mutex> lock(buffer_mutex_);
                if (audio_buffer_.size() >= BUFFER_SIZE) {
                    std::copy(audio_buffer_.begin(), audio_buffer_.begin() + BUFFER_SIZE, 
                             playback_buffer.begin());
                    audio_buffer_.erase(audio_buffer_.begin(), audio_buffer_.begin() + BUFFER_SIZE);
                } else {
                    playback_buffer.assign(BUFFER_SIZE, 0.0f);
                }
            }

            // Play audio if enabled
            if (is_playing_) {
                if (test_mode_) {
                    // In test mode, just log the audio data
                    RCLCPP_DEBUG(this->get_logger(), "Test mode: Would play %zu samples", playback_buffer.size());
                } else {
                    if (portaudio_output_stream_) {
                        PaError err = Pa_WriteStream(portaudio_output_stream_, playback_buffer.data(), BUFFER_SIZE);
                        if (err != paNoError && err != paOutputUnderflowed) {
                            RCLCPP_WARN(this->get_logger(), "PortAudio write error: %s", Pa_GetErrorText(err));
                        }
                    } else if (pcm_output_handle_) {
                        int err = snd_pcm_writei(pcm_output_handle_, playback_buffer.data(), BUFFER_SIZE);
                        if (err < 0) {
                            if (err == -EPIPE) {
                                RCLCPP_WARN(this->get_logger(), "ALSA output buffer underrun");
                                snd_pcm_prepare(pcm_output_handle_);
                            } else {
                                RCLCPP_WARN(this->get_logger(), "ALSA write error: %s", snd_strerror(err));
                            }
                        }
                    }
                }
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(23)); // ~44Hz update rate
        }
    }

    void update_ui()
    {
        if (!cv_windows_available_) {
            // In headless mode, just check for shutdown
            if (!rclcpp::ok()) {
                RCLCPP_INFO(this->get_logger(), "Shutdown requested");
                return;
            }
            return;
        }

        // Check for keyboard input
        int key = cv::waitKey(1) & 0xFF;
        
        if (key == 'p' || key == 'P') {
            is_playing_ = !is_playing_;
            RCLCPP_INFO(this->get_logger(), "Playback %s", is_playing_ ? "enabled" : "disabled");
        } else if (key == 'q' || key == 'Q' || key == 27) { // ESC key
            RCLCPP_INFO(this->get_logger(), "Quit requested");
            rclcpp::shutdown();
            return;
        }

        // Update visualization window
        {
            std::lock_guard<std::mutex> lock(vis_mutex_);
            if (has_new_visualization_ && !current_visualization_.empty()) {
                try {
                    cv::imshow("Audio Visualization", current_visualization_);
                    has_new_visualization_ = false;
                } catch (const cv::Exception& e) {
                    RCLCPP_WARN(this->get_logger(), "Failed to show visualization: %s", e.what());
                }
            }
        }

        // Update control panel
        update_control_panel();
    }

    void update_control_panel()
    {
        if (!cv_windows_available_) {
            return;
        }

        const int width = 400;
        const int height = 300;
        
        cv::Mat control_panel = cv::Mat::zeros(height, width, CV_8UC3);
        
        // Background
        control_panel = cv::Scalar(50, 50, 50);
        
        // Title
        cv::putText(control_panel, "Audio Control Panel", cv::Point(20, 30), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);
        
        // Test mode indicator
        if (test_mode_) {
            cv::putText(control_panel, "TEST MODE", cv::Point(250, 30), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
        }
        
        // Status indicators
        cv::Scalar status_color = is_playing_ ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
        cv::circle(control_panel, cv::Point(50, 70), 10, status_color, -1);
        cv::putText(control_panel, is_playing_ ? "PLAYING" : "STOPPED", 
                   cv::Point(70, 75), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
        
        // Audio levels
        cv::putText(control_panel, "RMS Level:", cv::Point(20, 110), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
        
        // RMS level bar
        int rms_bar_width = static_cast<int>(current_rms_ * 200);
        cv::rectangle(control_panel, cv::Point(20, 120), cv::Point(20 + rms_bar_width, 140), 
                     cv::Scalar(0, 255, 0), -1);
        cv::rectangle(control_panel, cv::Point(20, 120), cv::Point(220, 140), 
                     cv::Scalar(255, 255, 255), 1);
        
        // Peak level
        cv::putText(control_panel, "Peak Level:", cv::Point(20, 170), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
        
        // Peak level bar
        int peak_bar_width = static_cast<int>(current_peak_ * 200);
        cv::rectangle(control_panel, cv::Point(20, 180), cv::Point(20 + peak_bar_width, 200), 
                     cv::Scalar(0, 0, 255), -1);
        cv::rectangle(control_panel, cv::Point(20, 180), cv::Point(220, 200), 
                     cv::Scalar(255, 255, 255), 1);
        
        // Controls
        cv::putText(control_panel, "Controls:", cv::Point(20, 230), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
        cv::putText(control_panel, "P - Toggle Playback", cv::Point(20, 250), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
        cv::putText(control_panel, "Q - Quit", cv::Point(20, 270), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
        
        try {
            cv::imshow("Audio Control Panel", control_panel);
        } catch (const cv::Exception& e) {
            RCLCPP_WARN(this->get_logger(), "Failed to show control panel: %s", e.what());
        }
    }

    void cleanup_audio()
    {
        if (portaudio_output_stream_) {
            Pa_StopStream(portaudio_output_stream_);
            Pa_CloseStream(portaudio_output_stream_);
            Pa_Terminate();
        }
        
        if (pcm_output_handle_) {
            snd_pcm_close(pcm_output_handle_);
        }
    }
    
    void keyboard_input_loop()
    {
        char input;
        while (rclcpp::ok()) {
            input = getchar();
            
            switch (input) {
                case 'p':
                case 'P':
                    is_playing_ = !is_playing_;
                    if (is_playing_) {
                        RCLCPP_INFO(this->get_logger(), "🔊 Playback ON");
                    } else {
                        RCLCPP_INFO(this->get_logger(), "🔇 Playback OFF");
                    }
                    break;
                    
                case 't':
                case 'T':
                    simulate_playback_ = !simulate_playback_;
                    if (simulate_playback_) {
                        RCLCPP_INFO(this->get_logger(), "🎵 Playback simulation ON (frequency: %.1f Hz)", playback_frequency_.load());
                    } else {
                        RCLCPP_INFO(this->get_logger(), "🔇 Playback simulation OFF");
                    }
                    break;
                    
                case '1':
                    playback_frequency_ = 220.0; // A3
                    RCLCPP_INFO(this->get_logger(), "🎵 Playback frequency set to 220.0 Hz (A3)");
                    break;
                case '2':
                    playback_frequency_ = 246.94; // B3
                    RCLCPP_INFO(this->get_logger(), "🎵 Playback frequency set to 246.9 Hz (B3)");
                    break;
                case '3':
                    playback_frequency_ = 261.63; // C4
                    RCLCPP_INFO(this->get_logger(), "🎵 Playback frequency set to 261.6 Hz (C4)");
                    break;
                case '4':
                    playback_frequency_ = 293.66; // D4
                    RCLCPP_INFO(this->get_logger(), "🎵 Playback frequency set to 293.7 Hz (D4)");
                    break;
                case '5':
                    playback_frequency_ = 329.63; // E4
                    RCLCPP_INFO(this->get_logger(), "🎵 Playback frequency set to 329.6 Hz (E4)");
                    break;
                case '6':
                    playback_frequency_ = 349.23; // F4
                    RCLCPP_INFO(this->get_logger(), "🎵 Playback frequency set to 349.2 Hz (F4)");
                    break;
                case '7':
                    playback_frequency_ = 392.00; // G4
                    RCLCPP_INFO(this->get_logger(), "🎵 Playback frequency set to 392.0 Hz (G4)");
                    break;
                case '8':
                    playback_frequency_ = 440.00; // A4
                    RCLCPP_INFO(this->get_logger(), "🎵 Playback frequency set to 440.0 Hz (A4)");
                    break;
                case '9':
                    playback_frequency_ = 493.88; // B4
                    RCLCPP_INFO(this->get_logger(), "🎵 Playback frequency set to 493.9 Hz (B4)");
                    break;
                    
                case 'q':
                case 'Q':
                    RCLCPP_INFO(this->get_logger(), "Quitting...");
                    rclcpp::shutdown();
                    return;
                    
                default:
                    break;
            }
        }
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AudioSubscriber>());
    rclcpp::shutdown();
    return 0;
}
