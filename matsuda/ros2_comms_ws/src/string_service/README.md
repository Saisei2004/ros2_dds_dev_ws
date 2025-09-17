# String Service Package

ROS2 Jazzy用の文字列サービス通信パッケージです。サービスサーバーとクライアントを使用して文字列メッセージのリクエスト・レスポンス通信を行います。

## パッケージ構成

```
string_service/
├── CMakeLists.txt                    # ビルド設定
├── package.xml                       # パッケージ設定
├── README.md                         # このファイル
├── src/
│   ├── string_service_server.cpp     # サービスサーバー
│   └── string_service_client.cpp     # サービスクライアント
└── include/string_service/           # ヘッダーファイル用
```

## 機能

### string_service_server
- **サービス名**: `string_service`
- **サービス型**: `std_srvs/SetBool`
- **動作**: リクエストを受信し、処理結果をレスポンスとして返す
- **実行ファイル**: `string_service_server`

### string_service_client
- **サービス名**: `string_service`
- **サービス型**: `std_srvs/SetBool`
- **動作**: 3秒間隔でサービスにリクエストを送信し、レスポンスを受信
- **実行ファイル**: `string_service_client`

## サービス仕様

### リクエスト (std_srvs/SetBool)
```yaml
bool data    # true/falseの値
```

### レスポンス (std_srvs/SetBool)
```yaml
bool success          # 処理成功フラグ
string message        # 処理結果メッセージ
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
colcon build --packages-select string_service
```

4. ワークスペースをソース:
```bash
source install/setup.bash
```

## 使用方法

### 1. サービスサーバーの実行

```bash
ros2 run string_service string_service_server
```

**出力例:**
```
[INFO] [1757919663.186236905] [string_service_server]: String Service Server started, waiting for requests...
[INFO] [1757919667.245493881] [string_service_server]: Received request: data=true
[INFO] [1757919667.245493881] [string_service_server]: Sending response: success=true, message='Server processed: TRUE at time: 1757919667.245493'
```

### 2. サービスクライアントの実行

別のターミナルで以下を実行:

```bash
ros2 run string_service string_service_client
```

**出力例:**
```
[INFO] [1757919663.186236905] [string_service_client]: String Service Client started
[INFO] [1757919666.186438877] [string_service_client]: Sending request: data=true
[INFO] [1757919666.245493881] [string_service_client]: Received response: success=true, message='Server processed: TRUE at time: 1757919666.245493'
[INFO] [1757919669.186426306] [string_service_client]: Sending request: data=false
[INFO] [1757919669.245493881] [string_service_client]: Received response: success=true, message='Server processed: FALSE at time: 1757919669.245493'
```

### 3. バックグラウンド実行

サービスサーバーをバックグラウンドで実行:
```bash
ros2 run string_service string_service_server &
```

停止する場合:
```bash
pkill -f string_service_server
```

## サービス情報の確認

### アクティブなサービス一覧
```bash
ros2 service list
```

### サービスの詳細情報
```bash
ros2 service info string_service
```

### サービスの型情報
```bash
ros2 interface show std_srvs/srv/SetBool
```

### 手動でサービスを呼び出し
```bash
# trueを送信
ros2 service call string_service std_srvs/srv/SetBool "{data: true}"

# falseを送信
ros2 service call string_service std_srvs/srv/SetBool "{data: false}"
```

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
colcon build --packages-select string_service --cmake-clean-cache
```

### サービスが見つからない場合
1. サービスサーバーが起動しているか確認
2. `source install/setup.bash` が実行されているか確認
3. `ros2 service list` でサービスが存在するか確認
4. サービス名が正しいか確認（`string_service`）

### クライアントがタイムアウトする場合
```bash
# サービスサーバーの状態を確認
ros2 node list
ros2 service list | grep string

# サービスサーバーを再起動
pkill -f string_service_server
ros2 run string_service string_service_server
```

### ノードが見つからない場合
```bash
# パッケージのインストール状況を確認
ros2 pkg list | grep string_service

# 実行ファイルの存在を確認
ls -la install/string_service/lib/string_service/
```

## ライセンス

MIT License

## 作成者

matsuda
