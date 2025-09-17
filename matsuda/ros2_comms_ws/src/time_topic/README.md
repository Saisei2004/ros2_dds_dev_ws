# Time Topic Package

ROS2 Jazzy用の時間情報トピック通信パッケージです。パブリッシャーとサブスクライバーを使用して時刻メッセージの送受信を行います。

## パッケージ構成

```
time_topic/
├── CMakeLists.txt              # ビルド設定
├── package.xml                 # パッケージ設定
├── README.md                   # このファイル
├── src/
│   ├── time_publisher.cpp      # 時間パブリッシャー
│   └── time_subscriber.cpp     # 時間サブスクライバー
└── include/time_topic/         # ヘッダーファイル用
```

## 機能

### time_publisher
- **トピック名**: `time_topic`
- **メッセージ型**: `builtin_interfaces/Time`
- **動作**: 1秒間隔で現在時刻を送信
- **実行ファイル**: `time_publisher`

### time_subscriber
- **トピック名**: `time_topic`
- **メッセージ型**: `builtin_interfaces/Time`
- **動作**: 受信した時刻をコンソールに出力し、統計情報を計算
- **実行ファイル**: `time_subscriber`

## 時間情報機能

### 時刻形式
- **ROS2時刻**: `builtin_interfaces/Time`メッセージ
  - `sec`: 秒（整数）
  - `nanosec`: ナノ秒（整数）
- **人間が読みやすい形式**: `YYYY-MM-DD HH:MM:SS`形式

### 統計機能
サブスクライバーは以下の統計情報を提供します：
- **受信メッセージ数**: 受信した総メッセージ数
- **時間スパン**: 最初のメッセージから最後のメッセージまでの時間
- **周波数**: メッセージの受信周波数（Hz）

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
colcon build --packages-select time_topic
```

4. ワークスペースをソース:
```bash
source install/setup.bash
```

## 使用方法

### 1. 時間パブリッシャーの実行

```bash
ros2 run time_topic time_publisher
```

**出力例:**
```
[INFO] [1757920468.672888278] [time_publisher]: Time Publisher started
[INFO] [1757920469.672889278] [time_publisher]: Publishing time: 2025-01-15 15:12:49 (sec: 1737023569, nanosec: 672889278)
[INFO] [1757920470.672889279] [time_publisher]: Publishing time: 2025-01-15 15:12:50 (sec: 1737023570, nanosec: 672889279)
[INFO] [1757920471.672889280] [time_publisher]: Publishing time: 2025-01-15 15:12:51 (sec: 1737023571, nanosec: 672889280)
```

### 2. 時間サブスクライバーの実行

別のターミナルで以下を実行:

```bash
ros2 run time_topic time_subscriber
```

**出力例:**
```
[INFO] [1757920468.672888278] [time_subscriber]: Time Subscriber started, waiting for messages...
[INFO] [1757920469.672889278] [time_subscriber]: Received time: 2025-01-15 15:12:49 (sec: 1737023569, nanosec: 672889278)
[INFO] [1757920470.672889279] [time_subscriber]: Received time: 2025-01-15 15:12:50 (sec: 1737023570, nanosec: 672889279)
[INFO] [1757920477.672889280] [time_subscriber]: Statistics - Messages: 10, Time span: 9.00 sec, Frequency: 1.00 Hz
```

### 3. バックグラウンド実行

パブリッシャーをバックグラウンドで実行:
```bash
ros2 run time_topic time_publisher &
```

停止する場合:
```bash
pkill -f time_publisher
```

## トピック情報の確認

### アクティブなトピック一覧
```bash
ros2 topic list
```

### トピックの詳細情報
```bash
ros2 topic info time_topic
```

### トピックの内容を監視
```bash
ros2 topic echo time_topic
```

### トピックの周波数を確認
```bash
ros2 topic hz time_topic
```

## 時刻の特性

- **送信間隔**: 1秒間隔
- **精度**: ナノ秒精度
- **形式**: ROS2標準の`builtin_interfaces/Time`
- **統計計算**: リアルタイムで周波数と時間スパンを計算

## 時刻変換機能

### ROS2時刻から人間が読みやすい形式へ
- `builtin_interfaces/Time` → `YYYY-MM-DD HH:MM:SS`形式
- システム時刻との同期
- タイムゾーン対応（ローカル時刻）

### 統計計算
- **時間スパン**: 最初と最後のメッセージ間の時間差
- **周波数**: メッセージ受信の周波数（Hz）
- **精度**: ナノ秒レベルでの時間測定

## 依存関係

- `rclcpp`: ROS2 C++クライアントライブラリ
- `builtin_interfaces`: ROS2標準のインターフェース型
- `ament_cmake`: ビルドシステム

## トラブルシューティング

### ビルドエラーが発生する場合
```bash
# 依存関係を再インストール
rosdep install --from-paths src --ignore-src -r -y

# クリーンビルド
colcon build --packages-select time_topic --cmake-clean-cache
```

### メッセージが受信されない場合
1. 両方のノードが同じワークスペースでビルドされているか確認
2. `source install/setup.bash` が実行されているか確認
3. `ros2 topic list` でトピックが存在するか確認

### ノードが見つからない場合
```bash
# パッケージのインストール状況を確認
ros2 pkg list | grep time_topic

# 実行ファイルの存在を確認
ls -la install/time_topic/lib/time_topic/
```

### 統計情報が表示されない場合
- サブスクライバーは10回のメッセージ受信ごとに統計情報を表示します
- 十分な数のメッセージを受信しているか確認してください

### 時刻が正しく表示されない場合
- システム時刻が正しく設定されているか確認
- タイムゾーンの設定を確認

## 拡張のアイデア

- 異なる時間間隔の設定
- カスタム時刻形式の追加
- 時刻同期機能
- 複数のタイムゾーン対応
- 時刻データのファイル出力
- リアルタイムグラフ表示

## ライセンス

MIT License

## 作成者

matsuda
