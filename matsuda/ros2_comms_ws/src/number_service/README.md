# Number Service Package

ROS2 Jazzy用の数値サービス通信パッケージです。サービスサーバーとクライアントを使用して数値処理のリクエスト・レスポンス通信を行います。

## パッケージ構成

```
number_service/
├── CMakeLists.txt                    # ビルド設定
├── package.xml                       # パッケージ設定
├── README.md                         # このファイル
├── src/
│   ├── number_service_server.cpp     # 数値サービスサーバー
│   └── number_service_client.cpp     # 数値サービスクライアント
└── include/number_service/           # ヘッダーファイル用
```

## 機能

### number_service_server
- **サービス名**: `number_service`
- **サービス型**: `std_srvs/SetBool`
- **動作**: 数値処理リクエストを受信し、統計計算またはソート処理を実行
- **実行ファイル**: `number_service_server`

### number_service_client
- **サービス名**: `number_service`
- **サービス型**: `std_srvs/SetBool`
- **動作**: 5秒間隔で統計計算またはソート処理をリクエスト
- **実行ファイル**: `number_service_client`

## サービス機能

### リクエスト処理
- **true (統計計算)**: 数値リストの統計情報を計算
  - カウント、合計、平均、最小値、最大値
- **false (ソート処理)**: 数値リストを昇順でソート

### サンプルデータ
サーバーは以下の数値リストを保持しています：
```
[10, 25, 35, 42, 58, 67, 73, 81, 92, 15]
```

## サービス仕様

### リクエスト (std_srvs/SetBool)
```yaml
bool data    # true=統計計算, false=ソート処理
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
colcon build --packages-select number_service
```

4. ワークスペースをソース:
```bash
source install/setup.bash
```

## 使用方法

### 1. 数値サービスサーバーの実行

```bash
ros2 run number_service number_service_server
```

**出力例:**
```
[INFO] [1757920107.928917746] [number_service_server]: Number Service Server started, waiting for requests...
[INFO] [1757920107.928917746] [number_service_server]: Available numbers: [10, 25, 35, 42, 58, 67, 73, 81, 92, 15]
[INFO] [1757920112.929132697] [number_service_server]: Received request: data=true
[INFO] [1757920112.929132697] [number_service_server]: Sending response: success=true, message='Statistics - Count: 10, Sum: 498, Average: 49.800000, Min: 10, Max: 92'
```

### 2. 数値サービスクライアントの実行

別のターミナルで以下を実行:

```bash
ros2 run number_service number_service_client
```

**出力例:**
```
[INFO] [1757920107.928917746] [number_service_client]: Number Service Client started
[INFO] [1757920112.929132697] [number_service_client]: Sending request: data=true (statistics)
[INFO] [1757920112.929132697] [number_service_client]: Received response: Statistics - Count: 10, Sum: 498, Average: 49.800000, Min: 10, Max: 92
[INFO] [1757920117.929106041] [number_service_client]: Sending request: data=false (sorting)
[INFO] [1757920117.929106041] [number_service_client]: Received response: Sorted numbers: [10, 15, 25, 35, 42, 58, 67, 73, 81, 92]
```

### 3. バックグラウンド実行

サービスサーバーをバックグラウンドで実行:
```bash
ros2 run number_service number_service_server &
```

停止する場合:
```bash
pkill -f number_service_server
```

### 4. 自動終了機能

クライアントは10回のリクエスト後に自動的に終了します。

## サービス情報の確認

### アクティブなサービス一覧
```bash
ros2 service list
```

### サービスの詳細情報
```bash
ros2 service info number_service
```

### サービスの型情報
```bash
ros2 interface show std_srvs/srv/SetBool
```

### 手動でサービスを呼び出し
```bash
# 統計計算をリクエスト
ros2 service call number_service std_srvs/srv/SetBool "{data: true}"

# ソート処理をリクエスト
ros2 service call number_service std_srvs/srv/SetBool "{data: false}"
```

## 数値処理機能

### 統計計算 (data: true)
- **カウント**: 数値の個数
- **合計**: 全数値の合計
- **平均**: 数値の平均値
- **最小値**: 最小の数値
- **最大値**: 最大の数値

### ソート処理 (data: false)
- **昇順ソート**: 数値を小さい順に並び替え
- **元データ保持**: 元の数値リストは変更されない

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
colcon build --packages-select number_service --cmake-clean-cache
```

### サービスが見つからない場合
1. サービスサーバーが起動しているか確認
2. `source install/setup.bash` が実行されているか確認
3. `ros2 service list` でサービスが存在するか確認
4. サービス名が正しいか確認（`number_service`）

### クライアントがタイムアウトする場合
```bash
# サービスサーバーの状態を確認
ros2 node list
ros2 service list | grep number

# サービスサーバーを再起動
pkill -f number_service_server
ros2 run number_service number_service_server
```

### ノードが見つからない場合
```bash
# パッケージのインストール状況を確認
ros2 pkg list | grep number_service

# 実行ファイルの存在を確認
ls -la install/number_service/lib/number_service/
```

## 拡張のアイデア

- カスタム数値リストの追加
- 異なる数値処理アルゴリズム
- 数値のファイル入出力
- リアルタイム数値更新
- 複数の数値セット管理

## ライセンス

MIT License

## 作成者

matsuda
