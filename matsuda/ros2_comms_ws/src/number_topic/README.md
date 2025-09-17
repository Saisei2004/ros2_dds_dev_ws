# Number Topic Package

ROS2 Jazzy用の数値トピック通信パッケージです。パブリッシャーとサブスクライバーを使用して数値メッセージの送受信を行います。

## パッケージ構成

```
number_topic/
├── CMakeLists.txt              # ビルド設定
├── package.xml                 # パッケージ設定
├── README.md                   # このファイル
├── src/
│   ├── number_publisher.cpp    # 数値パブリッシャー
│   └── number_subscriber.cpp   # 数値サブスクライバー
└── include/number_topic/       # ヘッダーファイル用
```

## 機能

### number_publisher
- **トピック名**: `number_topic`
- **メッセージ型**: `std_msgs/Int32`
- **動作**: 1秒間隔で1-100の乱数を生成して送信
- **実行ファイル**: `number_publisher`

### number_subscriber
- **トピック名**: `number_topic`
- **メッセージ型**: `std_msgs/Int32`
- **動作**: 受信した数値をコンソールに出力し、統計情報を計算
- **実行ファイル**: `number_subscriber`

## 統計機能

サブスクライバーは以下の統計情報を提供します：
- **受信メッセージ数**: 受信した総メッセージ数
- **合計値**: 受信した数値の合計
- **平均値**: 受信した数値の平均
- **最小値**: 受信した数値の最小値
- **最大値**: 受信した数値の最大値

統計情報は10回ごとに表示されます。

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
colcon build --packages-select number_topic
```

4. ワークスペースをソース:
```bash
source install/setup.bash
```

## 使用方法

### 1. 数値パブリッシャーの実行

```bash
ros2 run number_topic number_publisher
```

**出力例:**
```
[INFO] [1757919939.658510494] [number_publisher]: Number Publisher started
[INFO] [1757919940.658789545] [number_publisher]: Publishing number: 42
[INFO] [1757919941.658789546] [number_publisher]: Publishing number: 73
[INFO] [1757919942.658789547] [number_publisher]: Publishing number: 15
```

### 2. 数値サブスクライバーの実行

別のターミナルで以下を実行:

```bash
ros2 run number_topic number_subscriber
```

**出力例:**
```
[INFO] [1757919939.658510494] [number_subscriber]: Number Subscriber started, waiting for messages...
[INFO] [1757919940.658789545] [number_subscriber]: Received number: 42
[INFO] [1757919941.658789546] [number_subscriber]: Received number: 73
[INFO] [1757919942.658789547] [number_subscriber]: Received number: 15
[INFO] [1757919949.658789548] [number_subscriber]: Statistics - Count: 10, Sum: 523, Average: 52.30, Min: 3, Max: 98
```

### 3. バックグラウンド実行

パブリッシャーをバックグラウンドで実行:
```bash
ros2 run number_topic number_publisher &
```

停止する場合:
```bash
pkill -f number_publisher
```

## トピック情報の確認

### アクティブなトピック一覧
```bash
ros2 topic list
```

### トピックの詳細情報
```bash
ros2 topic info number_topic
```

### トピックの内容を監視
```bash
ros2 topic echo number_topic
```

### トピックの周波数を確認
```bash
ros2 topic hz number_topic
```

## 数値の範囲と特性

- **生成範囲**: 1-100の整数
- **送信間隔**: 1秒間隔
- **乱数生成**: `std::mt19937`を使用した高品質な乱数
- **統計計算**: リアルタイムで最小値、最大値、平均値を計算

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
colcon build --packages-select number_topic --cmake-clean-cache
```

### メッセージが受信されない場合
1. 両方のノードが同じワークスペースでビルドされているか確認
2. `source install/setup.bash` が実行されているか確認
3. `ros2 topic list` でトピックが存在するか確認

### ノードが見つからない場合
```bash
# パッケージのインストール状況を確認
ros2 pkg list | grep number_topic

# 実行ファイルの存在を確認
ls -la install/number_topic/lib/number_topic/
```

### 統計情報が表示されない場合
- サブスクライバーは10回のメッセージ受信ごとに統計情報を表示します
- 十分な数のメッセージを受信しているか確認してください

## 拡張のアイデア

- 異なる数値範囲の設定
- カスタムメッセージ型の使用
- 統計情報のファイル出力
- リアルタイムグラフ表示
- 複数のサブスクライバーでの負荷分散

## ライセンス

MIT License

## 作成者

matsuda
