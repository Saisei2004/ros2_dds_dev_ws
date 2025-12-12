# Audio Topic Package

ROS2パッケージで音声の入出力と視覚的なフィードバックを提供します。

## 概要

このパッケージは以下の機能を提供します：

- **音声入力**: マイクからの音声をリアルタイムで録音・分析
- **音声出力**: 入力音声に基づいて生成された音声を再生
- **視覚的フィードバック**: 音声の波形、RMS、ピークレベルをリアルタイム表示
- **周波数分析**: FFTを使用した周波数スペクトラム分析

## パッケージ構成

```
audio_topic/
├── src/
│   ├── audio_publisher.cpp    # 音声入力・録音ノード
│   └── audio_subscriber.cpp   # 音声出力・視覚表示ノード
├── CMakeLists.txt
├── package.xml
└── README.md
```

## ノード説明

### audio_publisher

**機能**: 音声入力・録音・分析

**公開トピック**:
- `audio_topic` (std_msgs/Float64MultiArray): 音声統計データ
- `audio_visualization` (sensor_msgs/Image): 音声可視化画像

**特徴**:
- PortAudio/ALSAによる音声入力
- リアルタイム音声分析（RMS、ピーク、周波数）
- FFTによる周波数スペクトラム計算
- 音声波形の可視化

### audio_subscriber

**機能**: 音声出力・視覚表示・制御

**購読トピック**:
- `audio_topic` (std_msgs/Float64MultiArray): 音声統計データ
- `audio_visualization` (sensor_msgs/Image): 音声可視化画像

**特徴**:
- 入力音声に基づく音声生成・再生
- リアルタイム視覚表示
- インタラクティブ制御パネル
- 音声レベルのリアルタイム表示

## 必要な依存関係

### システム依存関係

```bash
# Ubuntu/Debian
sudo apt-get update
sudo apt-get install -y \
    libasound2-dev \
    portaudio19-dev \
    libsndfile1-dev \
    libfftw3-dev \
    libopencv-dev

# CentOS/RHEL
sudo yum install -y \
    alsa-lib-devel \
    portaudio-devel \
    libsndfile-devel \
    fftw-devel \
    opencv-devel
```

### ROS2依存関係

- `rclcpp`
- `std_msgs`
- `sensor_msgs`

## ビルド方法

```bash
cd /path/to/ros2_ws
colcon build --packages-select audio_topic
source install/setup.bash
```

## 使用方法

### 1. 基本的な使用方法

**ターミナル1**: 音声入力ノードを起動
```bash
ros2 run audio_topic audio_publisher
```

**ターミナル2**: 音声出力・視覚表示ノードを起動
```bash
ros2 run audio_topic audio_subscriber
```

### 2. 制御方法

#### audio_subscriberの制御

- **Pキー**: 音声再生のオン/オフ切り替え
- **Qキー**: プログラム終了
- **ESCキー**: プログラム終了

#### 視覚表示

- **Audio Visualization**: 音声波形、RMS、ピークレベルの表示
- **Audio Control Panel**: 再生状態、レベル表示、制御説明

### 3. 音声データの内容

#### audio_topic (std_msgs/Float64MultiArray)

```
data[0]: RMS値 (音声の平均的な大きさ)
data[1]: ピーク値 (音声の最大値)
data[2...]: 周波数スペクトラムデータ (FFT結果)
```

#### audio_visualization (sensor_msgs/Image)

- 音声波形の描画
- RMS・ピークレベルバー
- リアルタイム更新

## 技術仕様

### 音声パラメータ

- **サンプリングレート**: 44.1kHz
- **チャンネル数**: 1 (モノラル)
- **ビット深度**: 32bit float
- **バッファサイズ**: 1024サンプル

### FFT分析

- **FFTサイズ**: 1024ポイント
- **周波数分解能**: ~43Hz
- **最大周波数**: 22.05kHz

### 視覚表示

- **解像度**: 800x400 (波形表示)
- **更新レート**: 10Hz
- **色設定**: 
  - 波形: 緑色
  - RMS: 緑色バー
  - ピーク: 赤色バー

## トラブルシューティング

### 音声入力が動作しない場合

1. **オーディオデバイスの確認**
```bash
# ALSAデバイス一覧
aplay -l
arecord -l

# PortAudioデバイス確認
paplay /usr/share/sounds/alsa/Front_Left.wav
```

2. **権限の確認**
```bash
# audioグループに追加
sudo usermod -a -G audio $USER
# ログアウト・ログインが必要
```

3. **デバイスの使用状況確認**
```bash
# 使用中のプロセス確認
lsof /dev/snd/*
```

### 音声出力が動作しない場合

1. **音量設定の確認**
```bash
# ALSA音量設定
alsamixer
# または
amixer set Master 80%
```

2. **デフォルトデバイスの確認**
```bash
# デフォルトデバイス設定
aplay -D default /usr/share/sounds/alsa/Front_Left.wav
```

### ビルドエラーの場合

1. **依存関係の再インストール**
```bash
sudo apt-get install --reinstall libasound2-dev portaudio19-dev libsndfile1-dev libfftw3-dev
```

2. **CMakeキャッシュのクリア**
```bash
rm -rf build/audio_topic
colcon build --packages-select audio_topic
```

## カスタマイズ

### 音声パラメータの変更

`audio_publisher.cpp`と`audio_subscriber.cpp`の定数を変更：

```cpp
static constexpr int SAMPLE_RATE = 44100;  // サンプリングレート
static constexpr int BUFFER_SIZE = 1024;   // バッファサイズ
static constexpr int FFT_SIZE = 1024;      // FFTサイズ
```

### 視覚表示のカスタマイズ

- 色の変更: `cv::Scalar(B, G, R)`の値を調整
- 解像度の変更: `width`と`height`の値を調整
- 更新レートの変更: タイマー間隔を調整

## ライセンス

Apache-2.0

## 作成者

matsuda@junkers12.in.xack.co.jp
