# String Topic Package

ROS2 Jazzy用の文字列トピック通信パッケージです。パブリッシャーとサブスクライバーを使用して文字列メッセージの送受信を行います。

## パッケージ構成

```
string_topic/
├── CMakeLists.txt              # ビルド設定
├── package.xml                 # パッケージ設定
├── README.md                   # このファイル
├── src/
│   ├── string_publisher.cpp    # 文字列パブリッシャー
│   └── string_subscriber.cpp   # 文字列サブスクライバー
└── include/string_topic/       # ヘッダーファイル用
```

## 機能

### string_publisher
- **トピック名**: `string_topic`
- **メッセージ型**: `std_msgs/String`
- **動作**: 1秒間隔でタイムスタンプ付きの文字列メッセージを送信
- **実行ファイル**: `string_publisher`

### string_subscriber
- **トピック名**: `string_topic`
- **メッセージ型**: `std_msgs/String`
- **動作**: 受信した文字列メッセージをコンソールに出力
- **実行ファイル**: `string_subscriber`

## ビルド方法

1. ワークスペースのルートディレクトリに移動:
```bash
cd /home/matsuda/ros2_comms_ws
```

2. ROS2環境をセットアップ:
```bash
source /opt/ros/jazzy/setup.bash
```

3. パッケージをビルド:
```bash
colcon build --packages-select string_topic
```

4. ワークスペースをソース:
```bash
source install/setup.bash
```

## 使用方法

### 1. パブリッシャーの実行

```bash
ros2 run string_topic string_publisher
```

**出力例:**
```
[INFO] [1757919424.244915067] [string_publisher]: String Publisher started
[INFO] [1757919425.245115848] [string_publisher]: Publishing: 'Hello from string_publisher! Time: 1757919425.245052'
[INFO] [1757919426.245113261] [string_publisher]: Publishing: 'Hello from string_publisher! Time: 1757919426.245018'
```

### 2. サブスクライバーの実行

別のターミナルで以下を実行:

```bash
ros2 run string_topic string_subscriber
```

**出力例:**
```
[INFO] [1757919426.153345720] [string_subscriber]: String Subscriber started, waiting for messages...
[INFO] [1757919426.245493881] [string_subscriber]: I heard: 'Hello from string_publisher! Time: 1757919426.245018'
[INFO] [1757919427.245314110] [string_subscriber]: I heard: 'Hello from string_publisher! Time: 1757919427.245012'
```

### 3. バックグラウンド実行

パブリッシャーをバックグラウンドで実行:
```bash
ros2 run string_topic string_publisher &
```

停止する場合:
```bash
pkill -f string_publisher
```

## トピック情報の確認

### アクティブなトピック一覧
```bash
ros2 topic list
```

### トピックの詳細情報
```bash
ros2 topic info string_topic
```

### トピックの内容を監視
```bash
ros2 topic echo string_topic
```

## 依存関係

- `rclcpp`: ROS2 C++クライアントライブラリ
- `std_msgs`: 標準メッセージ型
- `ament_cmake`: ビルドシステム

## トラブルシューティング

### ビルドエラーが発生する場合
```bash
# 依存関係を再インストール
rosdep install --from-paths src --ignore-src -r -y

# クリーンビルド
colcon build --packages-select string_topic --cmake-clean-cache
```

### メッセージが受信されない場合
1. 両方のノードが同じワークスペースでビルドされているか確認
2. `source install/setup.bash` が実行されているか確認
3. `ros2 topic list` でトピックが存在するか確認

### ノードが見つからない場合
```bash
# パッケージのインストール状況を確認
ros2 pkg list | grep string_topic

# 実行ファイルの存在を確認
ls -la install/string_topic/lib/string_topic/
```

## ライセンス

MIT License

## 作成者

matsuda
