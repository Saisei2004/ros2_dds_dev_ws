# Time Service Package

ROS2 Jazzy用の時間サービス通信パッケージです。サービスサーバーとクライアントを使用して時間情報のリクエスト・レスポンス通信を行います。

## パッケージ構成

```
time_service/
├── CMakeLists.txt                    # ビルド設定
├── package.xml                       # パッケージ設定
├── README.md                         # このファイル
├── src/
│   ├── time_service_server.cpp       # 時間サービスサーバー
│   └── time_service_client.cpp       # 時間サービスクライアント
└── include/time_service/             # ヘッダーファイル用
```

## 機能

### time_service_server
- **サービス名**: `time_service`
- **サービス型**: `std_srvs/SetBool`
- **動作**: 時間情報リクエストを受信し、詳細な時刻情報またはフォーマット済み時刻を返す
- **実行ファイル**: `time_service_server`

### time_service_client
- **サービス名**: `time_service`
- **サービス型**: `std_srvs/SetBool`
- **動作**: 3秒間隔で詳細時刻情報またはフォーマット済み時刻をリクエスト
- **実行ファイル**: `time_service_client`

## サービス機能

### リクエスト処理
- **true (詳細時刻情報)**: 包括的な時刻情報を提供
  - フォーマット済み時刻（YYYY-MM-DD HH:MM:SS）
  - Unixタイムスタンプ
  - ナノ秒情報
  - 総ナノ秒数
- **false (フォーマット済み時刻)**: 人間が読みやすい形式の現在時刻

## サービス仕様

### リクエスト (std_srvs/SetBool)
```yaml
bool data    # true=詳細時刻情報, false=フォーマット済み時刻
```

### レスポンス (std_srvs/SetBool)
```yaml
bool success          # 処理成功フラグ
string message        # 時刻情報メッセージ
```

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
colcon build --packages-select time_service
```

4. ワークスペースをソース:
```bash
source install/setup.bash
```

## 使用方法

### 1. 時間サービスサーバーの実行

```bash
ros2 run time_service time_service_server
```

**出力例:**
```
[INFO] [1757920637.643840080] [time_service_server]: Time Service Server started, waiting for requests...
[INFO] [1757920641.644029413] [time_service_server]: Received request: data=true
[INFO] [1757920641.644029413] [time_service_server]: Sending response: success=true, message='Detailed time info - Formatted: 2025-01-15 15:17:21, Unix timestamp: 1737023841, Nanoseconds: 644029413, Total nanoseconds: 1737023841644029413'
```

### 2. 時間サービスクライアントの実行

別のターミナルで以下を実行:

```bash
ros2 run time_service time_service_client
```

**出力例:**
```
[INFO] [1757920637.643840080] [time_service_client]: Time Service Client started
[INFO] [1757920641.644029413] [time_service_client]: Sending request: data=true (detailed time info)
[INFO] [1757920641.644029413] [time_service_client]: Received response: Detailed time info - Formatted: 2025-01-15 15:17:21, Unix timestamp: 1737023841, Nanoseconds: 644029413, Total nanoseconds: 1737023841644029413
[INFO] [1757920644.644028468] [time_service_client]: Sending request: data=false (formatted time)
[INFO] [1757920644.644028468] [time_service_client]: Received response: Current time: 2025-01-15 15:17:24
```

### 3. バックグラウンド実行

サービスサーバーをバックグラウンドで実行:
```bash
ros2 run time_service time_service_server &
```

停止する場合:
```bash
pkill -f time_service_server
```

### 4. 自動終了機能

クライアントは8回のリクエスト後に自動的に終了します。

## サービス情報の確認

### アクティブなサービス一覧
```bash
ros2 service list
```

### サービスの詳細情報
```bash
ros2 service info time_service
```

### サービスの型情報
```bash
ros2 interface show std_srvs/srv/SetBool
```

### 手動でサービスを呼び出し
```bash
# 詳細時刻情報をリクエスト
ros2 service call time_service std_srvs/srv/SetBool "{data: true}"

# フォーマット済み時刻をリクエスト
ros2 service call time_service std_srvs/srv/SetBool "{data: false}"
```

## 時間情報機能

### 詳細時刻情報 (data: true)
- **フォーマット済み時刻**: `YYYY-MM-DD HH:MM:SS`形式
- **Unixタイムスタンプ**: 1970年1月1日からの秒数
- **ナノ秒**: 秒以下のナノ秒部分
- **総ナノ秒数**: 完全なナノ秒精度のタイムスタンプ

### フォーマット済み時刻 (data: false)
- **現在時刻**: 人間が読みやすい形式の現在時刻
- **形式**: `YYYY-MM-DD HH:MM:SS`

## 時刻変換機能

### ROS2時刻処理
- **高精度**: ナノ秒精度での時刻取得
- **システム同期**: システム時刻との同期
- **タイムゾーン対応**: ローカル時刻での表示

### 時刻フォーマット
- **標準形式**: ISO 8601準拠の日時形式
- **タイムスタンプ**: Unix標準の秒単位タイムスタンプ
- **ナノ秒精度**: 高精度な時刻情報

## 依存関係

- `rclcpp`: ROS2 C++クライアントライブラリ
- `std_srvs`: 標準サービス型
- `ament_cmake`: ビルドシステム

## トラブルシューティング

### ビルドエラーが発生する場合
```bash
# 依存関係を再インストール
rosdep install --from-paths src --ignore-src -r -y

# クリーンビルド
colcon build --packages-select time_service --cmake-clean-cache
```

### サービスが見つからない場合
1. サービスサーバーが起動しているか確認
2. `source install/setup.bash` が実行されているか確認
3. `ros2 service list` でサービスが存在するか確認
4. サービス名が正しいか確認（`time_service`）

### クライアントがタイムアウトする場合
```bash
# サービスサーバーの状態を確認
ros2 node list
ros2 service list | grep time

# サービスサーバーを再起動
pkill -f time_service_server
ros2 run time_service time_service_server
```

### ノードが見つからない場合
```bash
# パッケージのインストール状況を確認
ros2 pkg list | grep time_service

# 実行ファイルの存在を確認
ls -la install/time_service/lib/time_service/
```

### 時刻が正しく表示されない場合
- システム時刻が正しく設定されているか確認
- タイムゾーンの設定を確認
- システムクロックの同期状況を確認

## 拡張のアイデア

- 異なるタイムゾーンの対応
- カスタム時刻形式の追加
- 時刻同期機能
- 複数の時刻ソース対応
- 時刻データのファイル出力
- リアルタイム時刻更新
- 時刻比較機能

## ライセンス

MIT License

## 作成者

matsuda
