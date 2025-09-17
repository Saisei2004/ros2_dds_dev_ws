# image_service パッケージ

ROS2を使用した画像サービス通信パッケージです。画像処理のリクエスト・レスポンス機能を提供します。

## 機能

- **image_service_server**: 画像統計情報とフィルタリング情報を提供するサービスサーバー
- **image_service_client**: 画像サービスを呼び出すクライアント

## 依存関係

- `rclcpp`: ROS2 C++クライアントライブラリ
- `std_srvs`: 標準サービス型
- `sensor_msgs`: センサーメッセージ型
- `std_msgs`: 標準メッセージ型

## ビルド方法

```bash
# ワークスペースディレクトリに移動
cd /home/matsuda/ros2_comms_ws

# ワークスペースをビルド
colcon build --packages-select image_service

# 環境をセットアップ
source install/setup.bash
```

## 使用方法

### 1. サービスサーバーの起動

```bash
# ワークスペースの環境をセットアップ
source /home/matsuda/ros2_comms_ws/install/setup.bash

# 画像サービスサーバーを起動
ros2 run image_service image_service_server
```

**期待される出力:**
```
[INFO] [image_service_server]: 画像サービスサーバーを開始しました
[INFO] [image_service_server]: サービス名: image_service
[INFO] [image_service_server]: 利用可能な画像処理:
[INFO] [image_service_server]:   - リクエスト.data = true  : 画像統計情報取得
[INFO] [image_service_server]:   - リクエスト.data = false : 画像フィルタリング情報取得
```

### 2. サービスクライアントの起動

**別のターミナルで:**
```bash
# ワークスペースの環境をセットアップ
source /home/matsuda/ros2_comms_ws/install/setup.bash

# 画像統計情報を取得
ros2 run image_service image_service_client stats

# または画像フィルタリング情報を取得
ros2 run image_service image_service_client filter

# または両方の情報を取得
ros2 run image_service image_service_client both
```

**期待される出力（統計情報）:**
```
[INFO] [image_service_client]: 画像サービスクライアントを開始しました
[INFO] [image_service_client]: 画像統計情報リクエストを送信中...
[INFO] [image_service_client]: === 画像統計情報レスポンス ===
[INFO] [image_service_client]: === 画像統計情報 ===
[INFO] [image_service_client]: 画像サイズ: 640x480
[INFO] [image_service_client]: チャンネル数: 3 (BGR)
[INFO] [image_service_client]: 平均値 (B,G,R): (127.5, 127.5, 127.5)
[INFO] [image_service_client]: 標準偏差 (B,G,R): (45.2, 45.2, 45.2)
[INFO] [image_service_client]: 総画素数: 307200
[INFO] [image_service_client]: データサイズ: 921600 bytes
[INFO] [image_service_client]: ===============================
```

## サービス仕様

### サービス型
- **型**: `std_srvs/srv/SetBool`
- **名前**: `image_service`

### リクエスト
- **data**: `bool`
  - `true`: 画像統計情報を取得
  - `false`: 画像フィルタリング情報を取得

### レスポンス
- **success**: `bool` - 処理成功フラグ
- **message**: `string` - 詳細な情報（統計情報またはフィルタリング情報）

## 提供される画像処理機能

### 1. 画像統計情報（request.data = true）
- 画像サイズ情報
- チャンネル数
- 平均値（BGR各チャンネル）
- 標準偏差（BGR各チャンネル）
- 総画素数
- データサイズ

### 2. 画像フィルタリング情報（request.data = false）
- 元画像サイズ
- 適用可能なフィルタ一覧
- ガウシアンブラー（15x15カーネル）
- シャープニング（3x3カーネル）
- Cannyエッジ検出（閾値50, 150）
- モルフォロジー処理（楕円カーネル5x5）
- 処理時間
- メモリ使用量

## トラブルシューティング

### サービスが利用できない場合
```bash
# サービス一覧を確認
ros2 service list

# サービス情報を確認
ros2 service info /image_service

# サービス型を確認
ros2 interface show std_srvs/srv/SetBool
```

### サービスサーバーが起動しない場合
```bash
# 依存関係のインストール確認
sudo apt update
sudo apt install ros-jazzy-rclcpp ros-jazzy-std-srvs ros-jazzy-sensor-msgs
sudo apt install libopencv-dev
```

### サービスクライアントが応答しない場合
```bash
# サービスサーバーの状態確認
ros2 node list
ros2 node info /image_service_server

# サービス呼び出しテスト
ros2 service call /image_service std_srvs/srv/SetBool "{data: true}"
```

## テスト用コマンド

### 手動サービス呼び出し
```bash
# 統計情報取得
ros2 service call /image_service std_srvs/srv/SetBool "{data: true}"

# フィルタリング情報取得
ros2 service call /image_service std_srvs/srv/SetBool "{data: false}"
```

## 統計情報

### サーバー統計
- サービス呼び出し回数
- 処理時間
- エラー回数

### クライアント統計
- リクエスト送信回数
- レスポンス受信回数
- 成功/失敗率

## 注意事項

- サービスサーバーが起動している必要があります
- クライアントは非同期でサービスを呼び出します
- テスト画像は動的に生成されます（移動する図形を含む）
- 実際の画像処理時間はシミュレートされています

## ファイル構成

```
image_service/
├── CMakeLists.txt              # ビルド設定
├── package.xml                 # パッケージ設定
├── README.md                  # このファイル
└── src/
    ├── image_service_server.cpp # サービスサーバー
    └── image_service_client.cpp # サービスクライアント
```

## ライセンス

MIT License

## 作成者

matsuda

