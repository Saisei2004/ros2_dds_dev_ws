# image_topic パッケージ

ROS2を使用した画像トピック通信パッケージです。SSHトンネル経由でローカルPCから受信したカメラ画像をROS2トピックで処理し、処理済み画像をHTTPサーバーで配信します。

## 機能

- **image_publisher**: SSHトンネル経由でカメラ画像を受信し、ROS2トピックで送信
- **image_subscriber**: ROS2トピックを受信し、HTTPサーバーで処理済み画像を配信

## システム概要

```
ローカルPC → SSHトンネル → junkers12 → ROS2 → SSHトンネル → ローカルPC
    ↓              ↓              ↓        ↓         ↓              ↓
カメラ画像    ポート5009      image_    image_   ポート5010    処理済み画像
取得・送信                   publisher subscriber              受信・表示
```

## 依存関係

- `rclcpp`: ROS2 C++クライアントライブラリ
- `sensor_msgs`: センサーメッセージ型
- `OpenCV`: コンピュータビジョンライブラリ
- `curl`: HTTPリクエスト送信（system()コール使用）

## ビルド方法

```bash
# ワークスペースディレクトリに移動
cd /home/matsuda/ros2_comms_ws

# ワークスペースをビルド
colcon build --packages-select image_topic

# 環境をセットアップ
source install/setup.bash
```

## 完全な画像通信システムの起動方法

**⚠️ 必ず以下の順序で起動してください**

### 前提条件
- ローカルPC側で`camera_sender.py`が起動済み
- SSHトンネル1が設定済み（`ssh -N -T -R 5009:127.0.0.1:5001 junkers12`）

### 1. 画像パブリッシャー（SSH画像受信→ROS2送信）の起動

```bash
# ワークスペースの環境をセットアップ
cd /home/matsuda/ros2_comms_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# 画像パブリッシャーを起動
ros2 run image_topic image_publisher
```

**期待される出力:**
```
[INFO] [image_publisher]: 画像パブリッシャーを開始しました
[INFO] [image_publisher]: SSHで受信した画像をROS2トピックで送信します
[INFO] [image_publisher]: HTTPサーバー開始: http://127.0.0.1:5001/frame
[INFO] [image_publisher]: 画像をパブリッシュしました (フレーム数: 10, サイズ: 640x480)
```

### 2. 画像サブスクライバー（ROS2受信→HTTP配信）の起動

**別のターミナルで:**
```bash
# ワークスペースの環境をセットアップ
cd /home/matsuda/ros2_comms_ws
source install/setup.bash

# 画像サブスクライバーを起動
ros2 run image_topic image_subscriber
```

**期待される出力:**
```
[INFO] [image_subscriber]: 画像サブスクライバーを開始しました
[INFO] [image_subscriber]: トピック: image_topic
[INFO] [image_subscriber]: HTTPサーバー: http://127.0.0.1:5010/video
[INFO] [image_subscriber]: 画像受信 #1: 640x480, エンコーディング: bgr8, データサイズ: 921600 bytes
```

### 3. 動作確認

- **OpenCVウィンドウ**が開き、リアルタイムで画像が表示されます
- **ESCキー**または**Ctrl+C**で終了
- ローカルPC側で`local_ros2_image_viewer.py`を起動して処理済み画像を確認

## トピック情報

### ROS2トピック
- **名前**: `image_topic`
- **型**: `sensor_msgs/msg/Image`
- **説明**: カメラ画像（640x480、BGR8エンコーディング）
- **フレームレート**: 約10Hz（100ms間隔）

## 画像モード

### 自動切り替え機能
1. **カメラモード**: SSHトンネル経由でローカルPCのカメラ画像を受信
2. **テストモード**: カメラ画像が取得できない場合、テスト画像を生成

### カメラモードの特徴
- SSHトンネル経由で`http://127.0.0.1:5009/frame`から画像を取得
- curlコマンドを使用してHTTPリクエストを送信
- タイムアウト2秒で安全な通信を実現
- "CAMERA MODE"表示

### テストモードの特徴
- 灰色の背景
- 移動する赤い円
- 固定の色付き図形（緑、青、黄、紫の円）
- 白い枠線
- "TEST MODE"表示

## ポート使用状況

| ポート | 用途 | プログラム |
|--------|------|-----------|
| 5009 | SSHトンネル（カメラ画像受信） | image_publisher |
| 5010 | HTTPサーバー（処理済み画像配信） | image_subscriber |

## トラブルシューティング

### SSHトンネル接続エラー
```bash
# SSH接続テスト
ssh -A -J matsuda@blitz.xack.co.jp,matsuda@junkers1 matsuda@junkers12

# ポート転送テスト
telnet 127.0.0.1 5009
```

### ROS2ノードエラー
```bash
# ROS2トピック確認
ros2 topic list
ros2 topic echo /image_topic --once

# ノード状態確認
ros2 node list
ros2 node info /image_publisher
ros2 node info /image_subscriber
```

### 画像取得エラー
```bash
# ローカルカメラサーバー確認
curl -I http://127.0.0.1:5009/frame

# リモート画像配信確認
curl -I http://127.0.0.1:5010/video
```

### GStreamerエラー
```bash
# エラーが発生した場合、image_publisherを再起動
# またはテスト画像モードで動作確認
```

### OpenCVウィンドウが表示されない場合
- X11フォワーディングが有効か確認
- ディスプレイ環境変数が設定されているか確認
```bash
echo $DISPLAY
```

### ビルドエラーの場合
```bash
# 依存関係のインストール確認
sudo apt update
sudo apt install ros-jazzy-rclcpp ros-jazzy-sensor-msgs
sudo apt install libopencv-dev
```

## 統計情報

### パブリッシャー統計
- 送信フレーム数
- 画像サイズ（640x480）
- カメラモード/テストモードの表示

### サブスクライバー統計
- 受信フレーム数
- 経過時間
- 平均FPS
- 最後のフレームからの経過時間
- 画像データ統計（最小値、最大値、非ゼロ画素数）

## 注意事項

- ローカルPC側で`camera_sender.py`が起動している必要があります
- SSHトンネルが適切に設定されている必要があります
- カメラ画像が取得できない場合、自動的にテスト画像モードになります
- 画像は10Hz（100ms間隔）で送信されます
- OpenCVウィンドウはESCキーまたはCtrl+Cで終了してください
- 統計情報は5秒間隔で自動表示されます

## 関連ファイル

### ローカルPC側（/home/matsuda/local_src/）
- `camera_sender.py`: カメラ画像送信サーバー
- `local_ros2_image_viewer.py`: 処理済み画像表示
- `local_camera_viewer.py`: ローカルカメラ直接表示

## ファイル構成

```
image_topic/
├── CMakeLists.txt          # ビルド設定
├── package.xml             # パッケージ設定
├── README.md              # このファイル
└── src/
    ├── image_publisher.cpp # 画像送信ノード
    └── image_subscriber.cpp # 画像受信ノード
```
