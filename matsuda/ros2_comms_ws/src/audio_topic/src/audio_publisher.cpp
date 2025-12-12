#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <cmath>
#include <fstream>
#include <filesystem>
#include <sstream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/image.hpp"

// Audio libraries
#include <alsa/asoundlib.h>
#include <portaudio.h>
#include <sndfile.h>

// Visual feedback
#include <opencv2/opencv.hpp>

// FFT for frequency analysis
#include <fftw3.h>

using namespace std::chrono_literals;

class AudioPublisher : public rclcpp::Node
{
public:
    AudioPublisher() : Node("audio_publisher"), 
                      audio_buffer_(SAMPLE_RATE * BUFFER_DURATION),
                      fft_input_(FFT_SIZE),
                      fft_output_(FFT_SIZE / 2 + 1),
                      is_recording_(false),
                      test_mode_(false),
                      simulate_audio_(false),
                      simulation_frequency_(440.0),
                      file_mode_(false),
                      current_file_index_(0),
                      file_position_(0),
                      http_server_running_(false),
                      http_port_(5011)
    {
        // Create publisher for audio data
        audio_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "audio_topic", 10);
        
        // Create publisher for audio visualization
        visualization_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
            "audio_visualization", 10);

        // Initialize audio system
        if (!initialize_audio()) {
            RCLCPP_WARN(this->get_logger(), "Audio initialization failed - running in test mode");
            test_mode_ = true;
        } else {
            test_mode_ = false;
        }

        // Initialize FFT (disabled for stability)
        fft_plan_ = nullptr;
        RCLCPP_INFO(this->get_logger(), "FFT analysis disabled for stability");

        // Create timer for publishing audio data (50ms for smoother updates)
        timer_ = this->create_wall_timer(
            50ms, std::bind(&AudioPublisher::publish_audio_data, this));

        // Start recording thread
        recording_thread_ = std::thread(&AudioPublisher::recording_loop, this);
        
        // Start keyboard input thread for audio simulation
        keyboard_thread_ = std::thread(&AudioPublisher::keyboard_input_loop, this);
        
        // Initialize audio files
        initialize_audio_files();
        
        // Start HTTP server for audio streaming
        start_http_server();

        RCLCPP_INFO(this->get_logger(), "Audio Publisher started");
        RCLCPP_INFO(this->get_logger(), "Publishing to topic: audio_topic");
        RCLCPP_INFO(this->get_logger(), "Sample rate: %d Hz", SAMPLE_RATE);
        RCLCPP_INFO(this->get_logger(), "Buffer duration: %d seconds", BUFFER_DURATION);
        RCLCPP_INFO(this->get_logger(), "HTTP audio server: http://127.0.0.1:%d/audio", http_port_.load());
        RCLCPP_INFO(this->get_logger(), "Controls: 's' to simulate audio, 'f' for file mode, '1-9' to change frequency/file, 'q' to quit");
        RCLCPP_INFO(this->get_logger(), "Press Ctrl+C to stop");
    }

    ~AudioPublisher()
    {
        is_recording_ = false;
        if (recording_thread_.joinable()) {
            recording_thread_.join();
        }
        
        if (keyboard_thread_.joinable()) {
            keyboard_thread_.join();
        }
        
        if (http_server_thread_.joinable()) {
            http_server_running_ = false;
            http_server_thread_.join();
        }
        
        if (fft_plan_) {
            fftw_destroy_plan(fft_plan_);
        }
        
        cleanup_audio();
    }

private:
    // Audio parameters
    static constexpr int SAMPLE_RATE = 44100;
    static constexpr int BUFFER_DURATION = 1; // seconds
    static constexpr int FFT_SIZE = 1024;
    static constexpr int CHANNELS = 1;
    
    // Audio system
    snd_pcm_t* pcm_handle_;
    PaStream* portaudio_stream_;
    std::vector<float> audio_buffer_;
    std::mutex buffer_mutex_;
    std::atomic<bool> is_recording_;
    std::atomic<bool> test_mode_;
    std::thread recording_thread_;
    
    // Audio simulation
    std::atomic<bool> simulate_audio_;
    std::atomic<double> simulation_frequency_;
    std::thread keyboard_thread_;
    
    // Audio file playback
    std::atomic<bool> file_mode_;
    std::atomic<int> current_file_index_;
    std::atomic<size_t> file_position_;
    std::vector<std::string> audio_files_;
    std::vector<std::vector<float>> audio_file_data_;
    
    // HTTP server for audio streaming
    std::atomic<bool> http_server_running_;
    std::atomic<int> http_port_;
    std::thread http_server_thread_;
    
    // FFT for frequency analysis
    fftw_plan fft_plan_;
    std::vector<double> fft_input_;
    std::vector<double> fft_output_;
    
    // ROS2 publishers
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr audio_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr visualization_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool initialize_audio()
    {
        // Try PortAudio first (more portable)
        PaError err = Pa_Initialize();
        if (err != paNoError) {
            RCLCPP_ERROR(this->get_logger(), "PortAudio initialization failed: %s", Pa_GetErrorText(err));
            return initialize_alsa();
        }

        PaStreamParameters input_parameters;
        input_parameters.device = Pa_GetDefaultInputDevice();
        input_parameters.channelCount = CHANNELS;
        input_parameters.sampleFormat = paFloat32;
        input_parameters.suggestedLatency = Pa_GetDeviceInfo(input_parameters.device)->defaultLowInputLatency;
        input_parameters.hostApiSpecificStreamInfo = nullptr;

        err = Pa_OpenStream(&portaudio_stream_, &input_parameters, nullptr,
                           SAMPLE_RATE, paFramesPerBufferUnspecified,
                           paClipOff, nullptr, nullptr);

        if (err != paNoError) {
            RCLCPP_ERROR(this->get_logger(), "PortAudio stream open failed: %s", Pa_GetErrorText(err));
            Pa_Terminate();
            return initialize_alsa();
        }

        err = Pa_StartStream(portaudio_stream_);
        if (err != paNoError) {
            RCLCPP_ERROR(this->get_logger(), "PortAudio stream start failed: %s", Pa_GetErrorText(err));
            Pa_CloseStream(portaudio_stream_);
            Pa_Terminate();
            return initialize_alsa();
        }

        RCLCPP_INFO(this->get_logger(), "PortAudio initialized successfully");
        return true;
    }

    bool initialize_alsa()
    {
        int err = snd_pcm_open(&pcm_handle_, "default", SND_PCM_STREAM_CAPTURE, 0);
        if (err < 0) {
            RCLCPP_ERROR(this->get_logger(), "ALSA PCM open failed: %s", snd_strerror(err));
            return false;
        }

        snd_pcm_hw_params_t* hw_params;
        snd_pcm_hw_params_alloca(&hw_params);
        snd_pcm_hw_params_any(pcm_handle_, hw_params);
        snd_pcm_hw_params_set_access(pcm_handle_, hw_params, SND_PCM_ACCESS_RW_INTERLEAVED);
        snd_pcm_hw_params_set_format(pcm_handle_, hw_params, SND_PCM_FORMAT_FLOAT);
        unsigned int sample_rate = SAMPLE_RATE;
        snd_pcm_hw_params_set_rate_near(pcm_handle_, hw_params, &sample_rate, 0);
        snd_pcm_hw_params_set_channels(pcm_handle_, hw_params, CHANNELS);

        err = snd_pcm_hw_params(pcm_handle_, hw_params);
        if (err < 0) {
            RCLCPP_ERROR(this->get_logger(), "ALSA hw_params set failed: %s", snd_strerror(err));
            snd_pcm_close(pcm_handle_);
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "ALSA initialized successfully");
        return true;
    }

    void recording_loop()
    {
        is_recording_ = true;
        std::vector<float> buffer(SAMPLE_RATE / 10); // 100ms buffer
        
        while (is_recording_ && rclcpp::ok()) {
            if (test_mode_) {
                // Generate test audio signal
                generate_test_audio(buffer);
            } else {
                if (portaudio_stream_) {
                    // Use PortAudio
                    PaError err = Pa_ReadStream(portaudio_stream_, buffer.data(), 
                                              SAMPLE_RATE / 10);
                    if (err == paNoError) {
                        add_audio_to_buffer(buffer);
                    } else if (err != paInputOverflowed) {
                        RCLCPP_WARN(this->get_logger(), "PortAudio read error: %s", Pa_GetErrorText(err));
                    }
                } else if (pcm_handle_) {
                    // Use ALSA
                    int err = snd_pcm_readi(pcm_handle_, buffer.data(), buffer.size());
                    if (err < 0) {
                        if (err == -EPIPE) {
                            RCLCPP_WARN(this->get_logger(), "ALSA buffer overrun");
                            snd_pcm_prepare(pcm_handle_);
                        } else {
                            RCLCPP_WARN(this->get_logger(), "ALSA read error: %s", snd_strerror(err));
                        }
                    } else {
                        std::vector<float> alsa_buffer(buffer.begin(), buffer.begin() + err);
                        add_audio_to_buffer(alsa_buffer);
                    }
                }
            }
            
            std::this_thread::sleep_for(5ms); // Faster updates for smoother audio
        }
    }

    void publish_audio_data()
    {
        std::vector<float> current_buffer;
        
        // Copy current audio buffer
        {
            std::lock_guard<std::mutex> lock(buffer_mutex_);
            current_buffer = audio_buffer_;
        }

        if (current_buffer.empty()) {
            return;
        }

        // Calculate audio statistics
        float rms = 0.0f;
        float peak = 0.0f;
        for (float sample : current_buffer) {
            rms += sample * sample;
            peak = std::max(peak, std::abs(sample));
        }
        rms = std::sqrt(rms / current_buffer.size());

        // Create audio message
        auto audio_msg = std_msgs::msg::Float64MultiArray();
        audio_msg.data = {static_cast<double>(rms), static_cast<double>(peak)};
        
        // Add frequency analysis if we have enough data (disabled)
        // if (current_buffer.size() >= FFT_SIZE) {
        //     std::vector<double> freq_data = perform_fft_analysis(current_buffer);
        //     audio_msg.data.insert(audio_msg.data.end(), freq_data.begin(), freq_data.end());
        // }

        audio_publisher_->publish(audio_msg);

        // Create and publish visualization
        auto vis_msg = create_audio_visualization(current_buffer, rms, peak);
        visualization_publisher_->publish(vis_msg);

        RCLCPP_DEBUG(this->get_logger(), "Published audio data - RMS: %.4f, Peak: %.4f", rms, peak);
    }

    std::vector<double> perform_fft_analysis(const std::vector<float>& audio_data)
    {
        if (!fft_plan_ || audio_data.empty()) {
            return std::vector<double>();
        }

        // Copy audio data to FFT input (use latest FFT_SIZE samples)
        size_t start_idx = (audio_data.size() > FFT_SIZE) ? 
                          audio_data.size() - FFT_SIZE : 0;
        
        // Clear FFT input first
        std::fill(fft_input_.begin(), fft_input_.end(), 0.0);
        
        for (size_t i = 0; i < FFT_SIZE && start_idx + i < audio_data.size(); ++i) {
            fft_input_[i] = static_cast<double>(audio_data[start_idx + i]);
        }
        
        // Perform FFT
        try {
            fftw_execute(fft_plan_);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "FFT execution failed: %s", e.what());
            return std::vector<double>();
        }
        
        // Calculate magnitude spectrum
        std::vector<double> magnitudes;
        for (size_t i = 0; i < fft_output_.size(); ++i) {
            double real = fft_output_[i];
            double imag = (i < fft_output_.size()) ? fft_output_[i + 1] : 0.0;
            magnitudes.push_back(std::sqrt(real * real + imag * imag));
        }
        
        return magnitudes;
    }

    sensor_msgs::msg::Image create_audio_visualization(const std::vector<float>& audio_data, 
                                                      float rms, float peak)
    {
        const int width = 800;
        const int height = 400;
        
        cv::Mat vis_image = cv::Mat::zeros(height, width, CV_8UC3);
        
        // Draw waveform
        if (!audio_data.empty()) {
            std::vector<cv::Point> waveform_points;
            for (size_t i = 0; i < audio_data.size() && i < width; ++i) {
                int x = static_cast<int>((static_cast<double>(i) / audio_data.size()) * width);
                int y = static_cast<int>(height/2 - (audio_data[i] * height/2));
                waveform_points.push_back(cv::Point(x, y));
            }
            
            // Draw waveform line
            for (size_t i = 1; i < waveform_points.size(); ++i) {
                cv::line(vis_image, waveform_points[i-1], waveform_points[i], 
                        cv::Scalar(0, 255, 0), 1);
            }
        }
        
        // Draw RMS and Peak indicators
        int rms_bar_height = static_cast<int>(rms * height);
        int peak_bar_height = static_cast<int>(peak * height);
        
        // RMS bar (green)
        cv::rectangle(vis_image, cv::Point(10, height - rms_bar_height - 10), 
                     cv::Point(30, height - 10), cv::Scalar(0, 255, 0), -1);
        
        // Peak bar (red)
        cv::rectangle(vis_image, cv::Point(40, height - peak_bar_height - 10), 
                     cv::Point(60, height - 10), cv::Scalar(0, 0, 255), -1);
        
        // Add text labels
        cv::putText(vis_image, "RMS: " + std::to_string(rms), 
                   cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
        cv::putText(vis_image, "Peak: " + std::to_string(peak), 
                   cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
        
        // Convert to ROS2 Image message
        sensor_msgs::msg::Image image_msg;
        image_msg.header.stamp = this->now();
        image_msg.header.frame_id = "audio_visualization";
        image_msg.height = height;
        image_msg.width = width;
        image_msg.encoding = "bgr8";
        image_msg.is_bigendian = false;
        image_msg.step = width * 3;
        
        image_msg.data.resize(height * width * 3);
        std::memcpy(image_msg.data.data(), vis_image.data, height * width * 3);
        
        return image_msg;
    }

    void generate_test_audio(std::vector<float>& buffer)
    {
        static double time_counter = 0.0;
        static double envelope = 0.0;
        static double noise_filter = 0.0;
        double dt = 1.0 / SAMPLE_RATE;
        
        // ファイルモードの場合は音声ファイルから再生
        if (file_mode_ && !audio_file_data_.empty()) {
            generate_file_audio(buffer);
            return;
        }
        
        for (size_t i = 0; i < buffer.size(); ++i) {
            double sample = 0.0;
            
            if (simulate_audio_) {
                // Generate more realistic simulated audio
                double freq = simulation_frequency_.load();
                
                // Create envelope for more natural sound attack/decay
                double target_envelope = 0.8;
                envelope += (target_envelope - envelope) * 0.01; // Smooth envelope
                
                // Generate base tone with slight frequency modulation
                double freq_mod = freq + 2.0 * std::sin(2.0 * M_PI * 0.5 * time_counter); // Vibrato
                sample = envelope * 0.4 * std::sin(2.0 * M_PI * freq_mod * time_counter);
                
                // Add rich harmonics for more realistic timbre
                sample += envelope * 0.15 * std::sin(2.0 * M_PI * freq * 2.0 * time_counter);  // Octave
                sample += envelope * 0.08 * std::sin(2.0 * M_PI * freq * 3.0 * time_counter);  // Fifth
                sample += envelope * 0.04 * std::sin(2.0 * M_PI * freq * 4.0 * time_counter);  // Double octave
                
                // Add filtered noise for breathiness
                double noise = ((static_cast<float>(rand()) / RAND_MAX) - 0.5f);
                noise_filter = noise_filter * 0.95 + noise * 0.05; // Low-pass filter
                sample += envelope * 0.03 * noise_filter;
                
                // Add subtle amplitude modulation
                double amp_mod = 1.0 + 0.1 * std::sin(2.0 * M_PI * 3.7 * time_counter);
                sample *= amp_mod;
                
            } else {
                // Generate more realistic background ambient sound
                envelope = 0.3; // Lower background level
                
                // Create ambient soundscape with multiple frequencies
                sample += envelope * 0.08 * std::sin(2.0 * M_PI * 440.0 * time_counter);      // A4
                sample += envelope * 0.06 * std::sin(2.0 * M_PI * 523.25 * time_counter);     // C5
                sample += envelope * 0.04 * std::sin(2.0 * M_PI * 659.25 * time_counter);     // E5
                sample += envelope * 0.03 * std::sin(2.0 * M_PI * 220.0 * time_counter);      // A3
                
                // Add filtered noise for ambient texture
                double noise = ((static_cast<float>(rand()) / RAND_MAX) - 0.5f);
                noise_filter = noise_filter * 0.98 + noise * 0.02;
                sample += envelope * 0.02 * noise_filter;
                
                // Add slow amplitude modulation
                double amp_mod = 1.0 + 0.05 * std::sin(2.0 * M_PI * 0.2 * time_counter);
                sample *= amp_mod;
            }
            
            // Apply soft clipping to prevent harsh edges
            if (sample > 0.95) sample = 0.95;
            if (sample < -0.95) sample = -0.95;
            
            buffer[i] = static_cast<float>(sample);
            time_counter += dt;
        }
        
        // Add to buffer
        add_audio_to_buffer(buffer);
    }

    void add_audio_to_buffer(const std::vector<float>& buffer)
    {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        for (float sample : buffer) {
            audio_buffer_.push_back(sample);
            if (audio_buffer_.size() > SAMPLE_RATE * BUFFER_DURATION) {
                audio_buffer_.erase(audio_buffer_.begin());
            }
        }
    }

    void cleanup_audio()
    {
        if (portaudio_stream_) {
            Pa_StopStream(portaudio_stream_);
            Pa_CloseStream(portaudio_stream_);
            Pa_Terminate();
        }
        
        if (pcm_handle_) {
            snd_pcm_close(pcm_handle_);
        }
    }
    
    void keyboard_input_loop()
    {
        char input;
        while (rclcpp::ok() && is_recording_) {
            input = getchar();
            
            switch (input) {
                case 's':
                case 'S':
                    simulate_audio_ = !simulate_audio_;
                    if (simulate_audio_) {
                        file_mode_ = false; // シミュレーションモードに切り替え
                        RCLCPP_INFO(this->get_logger(), "🎵 Audio simulation ON (frequency: %.1f Hz)", simulation_frequency_.load());
                    } else {
                        RCLCPP_INFO(this->get_logger(), "🔇 Audio simulation OFF");
                    }
                    break;
                    
                case 'f':
                case 'F':
                    file_mode_ = !file_mode_;
                    if (file_mode_) {
                        simulate_audio_ = false; // ファイルモードに切り替え
                        file_position_ = 0; // ファイル位置をリセット
                        RCLCPP_INFO(this->get_logger(), "📁 File mode ON (file: %s)", 
                                   (audio_files_.empty() ? "none" : 
                                    std::filesystem::path(audio_files_[current_file_index_.load()]).filename().c_str()));
                    } else {
                        RCLCPP_INFO(this->get_logger(), "🔇 File mode OFF");
                    }
                    break;
                    
                case '1':
                case '2':
                case '3':
                case '4':
                case '5':
                case '6':
                case '7':
                case '8':
                case '9':
                    {
                        int key_num = input - '0';
                        if (file_mode_ && !audio_files_.empty()) {
                            // ファイルモード：ファイル選択
                            int file_idx = (key_num - 1) % audio_files_.size();
                            current_file_index_ = file_idx;
                            file_position_ = 0; // ファイル位置をリセット
                            RCLCPP_INFO(this->get_logger(), "📁 Selected file %d: %s", 
                                       file_idx + 1, 
                                       std::filesystem::path(audio_files_[file_idx]).filename().c_str());
                        } else {
                            // シミュレーションモード：周波数設定
                            double frequencies[] = {220.0, 246.94, 261.63, 293.66, 329.63, 349.23, 392.00, 440.00, 493.88};
                            const char* notes[] = {"A3", "B3", "C4", "D4", "E4", "F4", "G4", "A4", "B4"};
                            simulation_frequency_ = frequencies[key_num - 1];
                            RCLCPP_INFO(this->get_logger(), "🎵 Frequency set to %.1f Hz (%s)", 
                                       frequencies[key_num - 1], notes[key_num - 1]);
                        }
                    }
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
    
    void initialize_audio_files()
    {
        std::string audio_dir = "/home/matsuda/audio_samples";
        std::vector<std::string> file_extensions = {".wav", ".mp3", ".flac"};
        
        try {
            if (std::filesystem::exists(audio_dir)) {
                for (const auto& entry : std::filesystem::directory_iterator(audio_dir)) {
                    if (entry.is_regular_file()) {
                        std::string filename = entry.path().string();
                        std::string extension = entry.path().extension().string();
                        
                        // 対応する拡張子かチェック
                        bool supported = false;
                        for (const auto& ext : file_extensions) {
                            if (extension == ext) {
                                supported = true;
                                break;
                            }
                        }
                        
                        if (supported) {
                            audio_files_.push_back(filename);
                            RCLCPP_INFO(this->get_logger(), "Found audio file: %s", filename.c_str());
                        }
                    }
                }
                
                // ファイルを名前順にソート
                std::sort(audio_files_.begin(), audio_files_.end());
                
                // 音声ファイルデータを読み込み
                load_audio_files();
                
                RCLCPP_INFO(this->get_logger(), "Loaded %zu audio files", audio_files_.size());
            } else {
                RCLCPP_WARN(this->get_logger(), "Audio directory not found: %s", audio_dir.c_str());
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error initializing audio files: %s", e.what());
        }
    }
    
    void load_audio_files()
    {
        audio_file_data_.clear();
        
        for (const auto& filename : audio_files_) {
            try {
                SF_INFO sf_info;
                SNDFILE* sndfile = sf_open(filename.c_str(), SFM_READ, &sf_info);
                
                if (sndfile) {
                    std::vector<float> file_data(sf_info.frames * sf_info.channels);
                    sf_count_t frames_read = sf_readf_float(sndfile, file_data.data(), sf_info.frames);
                    
                    // モノラルに変換（必要に応じて）
                    std::vector<float> mono_data;
                    if (sf_info.channels == 1) {
                        mono_data = file_data;
                    } else {
                        // ステレオからモノラルに変換（左右平均）
                        mono_data.resize(sf_info.frames);
                        for (sf_count_t i = 0; i < sf_info.frames; ++i) {
                            mono_data[i] = (file_data[i * sf_info.channels] + 
                                          file_data[i * sf_info.channels + 1]) * 0.5f;
                        }
                    }
                    
                    // サンプルレート変換（44.1kHzに）
                    if (sf_info.samplerate != SAMPLE_RATE) {
                        mono_data = resample_audio(mono_data, sf_info.samplerate, SAMPLE_RATE);
                    }
                    
                    audio_file_data_.push_back(mono_data);
                    sf_close(sndfile);
                    
                    RCLCPP_INFO(this->get_logger(), "Loaded: %s (%zu samples)", 
                               filename.c_str(), mono_data.size());
                } else {
                    RCLCPP_WARN(this->get_logger(), "Could not open audio file: %s", filename.c_str());
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Error loading audio file %s: %s", 
                            filename.c_str(), e.what());
            }
        }
    }
    
    std::vector<float> resample_audio(const std::vector<float>& input, int input_rate, int output_rate)
    {
        if (input_rate == output_rate) {
            return input;
        }
        
        double ratio = static_cast<double>(output_rate) / input_rate;
        size_t output_size = static_cast<size_t>(input.size() * ratio);
        std::vector<float> output(output_size);
        
        for (size_t i = 0; i < output_size; ++i) {
            double src_index = i / ratio;
            size_t src_idx = static_cast<size_t>(src_index);
            double frac = src_index - src_idx;
            
            if (src_idx < input.size() - 1) {
                output[i] = input[src_idx] * (1.0 - frac) + input[src_idx + 1] * frac;
            } else {
                output[i] = input[input.size() - 1];
            }
        }
        
        return output;
    }
    
    void generate_file_audio(std::vector<float>& buffer)
    {
        int file_idx = current_file_index_.load();
        if (file_idx < 0 || file_idx >= static_cast<int>(audio_file_data_.size())) {
            return;
        }
        
        const auto& file_data = audio_file_data_[file_idx];
        size_t pos = file_position_.load();
        
        for (size_t i = 0; i < buffer.size(); ++i) {
            if (pos < file_data.size()) {
                buffer[i] = file_data[pos];
                pos++;
            } else {
                // ファイル終了 - ループ再生
                pos = 0;
                buffer[i] = file_data[pos];
                pos++;
            }
        }
        
        file_position_.store(pos);
    }
    
    void start_http_server()
    {
        http_server_thread_ = std::thread(&AudioPublisher::http_server_loop, this);
    }
    
    void http_server_loop()
    {
        int server_fd = socket(AF_INET, SOCK_STREAM, 0);
        if (server_fd < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create socket");
            return;
        }
        
        int opt = 1;
        setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
        
        struct sockaddr_in address;
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = INADDR_ANY;
        address.sin_port = htons(http_port_.load());
        
        if (bind(server_fd, (struct sockaddr*)&address, sizeof(address)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to bind socket to port %d", http_port_.load());
            close(server_fd);
            return;
        }
        
        if (listen(server_fd, 3) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to listen on socket");
            close(server_fd);
            return;
        }
        
        http_server_running_ = true;
        RCLCPP_INFO(this->get_logger(), "HTTP audio server started on port %d", http_port_.load());
        
        while (http_server_running_ && rclcpp::ok()) {
            struct sockaddr_in client_addr;
            socklen_t client_len = sizeof(client_addr);
            
            int client_fd = accept(server_fd, (struct sockaddr*)&client_addr, &client_len);
            if (client_fd < 0) {
                if (http_server_running_) {
                    RCLCPP_WARN(this->get_logger(), "Failed to accept connection");
                }
                continue;
            }
            
            // Handle client in a separate thread
            std::thread client_thread(&AudioPublisher::handle_http_client, this, client_fd);
            client_thread.detach();
        }
        
        close(server_fd);
        RCLCPP_INFO(this->get_logger(), "HTTP audio server stopped");
    }
    
    void handle_http_client(int client_fd)
    {
        char buffer[1024];
        int bytes_read = read(client_fd, buffer, sizeof(buffer) - 1);
        if (bytes_read <= 0) {
            close(client_fd);
            return;
        }
        
        buffer[bytes_read] = '\0';
        std::string request(buffer);
        
        // Simple HTTP request parsing
        if (request.find("GET /audio") != std::string::npos) {
            send_audio_response(client_fd);
        } else {
            send_404_response(client_fd);
        }
        
        close(client_fd);
    }
    
    void send_audio_response(int client_fd)
    {
        // HTTP headers
        std::string headers = 
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: audio/wav\r\n"
            "Transfer-Encoding: chunked\r\n"
            "Connection: close\r\n\r\n";
        
        write(client_fd, headers.c_str(), headers.length());
        
        // Send audio data in chunks
        std::vector<float> current_audio;
        {
            std::lock_guard<std::mutex> lock(buffer_mutex_);
            current_audio = audio_buffer_;
        }
        
        if (!current_audio.empty()) {
            // Convert float to 16-bit PCM
            std::vector<int16_t> pcm_data(current_audio.size());
            for (size_t i = 0; i < current_audio.size(); ++i) {
                pcm_data[i] = static_cast<int16_t>(current_audio[i] * 32767.0f);
            }
            
            // Send chunked data
            size_t chunk_size = 4096;
            for (size_t i = 0; i < pcm_data.size(); i += chunk_size) {
                size_t remaining = std::min(chunk_size, pcm_data.size() - i);
                
                // Chunk header
                std::stringstream chunk_header;
                chunk_header << std::hex << remaining * sizeof(int16_t) << "\r\n";
                write(client_fd, chunk_header.str().c_str(), chunk_header.str().length());
                
                // Chunk data
                write(client_fd, reinterpret_cast<const char*>(&pcm_data[i]), remaining * sizeof(int16_t));
                
                // Chunk footer
                write(client_fd, "\r\n", 2);
            }
        }
        
        // End chunk
        write(client_fd, "0\r\n\r\n", 5);
    }
    
    void send_404_response(int client_fd)
    {
        std::string response = 
            "HTTP/1.1 404 Not Found\r\n"
            "Content-Type: text/plain\r\n"
            "Content-Length: 13\r\n"
            "Connection: close\r\n\r\n"
            "404 Not Found";
        
        write(client_fd, response.c_str(), response.length());
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AudioPublisher>());
    rclcpp::shutdown();
    return 0;
}
