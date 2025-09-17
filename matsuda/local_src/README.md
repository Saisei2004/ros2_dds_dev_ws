# ローカルPC側プログラム

ローカルPCで実行する画像通信プログラム群です。

## ファイル構成

```
local_src/
├── camera_sender.py              # ローカルカメラ画像送信サーバー
├── local_camera_viewer.py        # ローカルカメラ画像直接表示
├── local_ros2_image_viewer.py    # ROS2処理済み画像受信・表示
├── requirements.txt              # Python依存関係
└── README.md                    # このファイル
```

## セットアップ

### 依存関係のインストール

```bash
cd /home/matsuda/local_src
pip install -r requirements.txt
```

### SSH鍵の設定

```bash
# SSHエージェントに鍵を登録
eval "$(ssh-agent -s)"
ssh-add ~/.ssh/id_ed25519.blitz
```

## 使用方法

### 完全な画像通信システム

#### 1. カメラ画像送信サーバー起動

```bash
cd /home/matsuda/local_src
python3 camera_sender.py
```

#### 2. SSHトンネル1設定（カメラ画像送信）

```bash
ssh -N -T -R 5009:127.0.0.1:5001 junkers12
```

#### 3. SSHトンネル2設定（処理済み画像受信）

```bash
ssh -N -T -R 5010:127.0.0.1:5010 junkers12
```

#### 4. 処理済み画像表示

```bash
cd /home/matsuda/local_src
python3 local_ros2_image_viewer.py
```

### テスト用: ローカルカメラ直接表示

```bash
cd /home/matsuda/local_src
python3 local_camera_viewer.py
```

## プログラム詳細

### camera_sender.py
- **機能**: ローカルカメラ画像を取得してHTTPサーバーで配信
- **ポート**: 5001（固定）
- **エンドポイント**: `/frame` (単体JPEG画像)
- **特徴**: カメラが利用できない場合はテスト画像を生成

### local_camera_viewer.py
- **機能**: ローカルカメラ画像を直接表示（ROS2処理なし）
- **用途**: カメラ動作確認用

### local_ros2_image_viewer.py
- **機能**: ROS2処理済み画像を受信・表示
- **接続先**: `http://127.0.0.1:5010/video`
- **特徴**: MJPEGストリームを解析してリアルタイム表示

## トラブルシューティング

### カメラが認識されない場合

```bash
# 利用可能なカメラデバイスを確認
ls /dev/video*

# カメラインデックスを変更
# コード内で camera_index を変更（0, 1, 2...）
```

### SSH接続エラー

```bash
# SSH接続テスト
ssh -A -J matsuda@blitz.xack.co.jp,matsuda@junkers1 matsuda@junkers12

# ポート転送テスト
telnet 127.0.0.1 5009
telnet 127.0.0.1 5010
```

### 画像取得エラー

```bash
# ローカルカメラサーバー確認
curl -I http://127.0.0.1:5001/frame

# リモート画像配信確認
curl -I http://127.0.0.1:5010/video
```

## システム仕様

### ポート使用状況

| ポート | 用途 | 方向 | プログラム |
|--------|------|------|-----------|
| 5001 | ローカルカメラ配信 | ローカル | camera_sender.py |
| 5009 | SSHトンネル（カメラ送信） | ローカル→リモート | SSH tunnel |
| 5010 | ROS2処理済み画像配信 | ローカル←リモート | SSH tunnel |

### 画像仕様

- **解像度**: 640x480
- **フォーマット**: BGR8
- **フレームレート**: 約10FPS
- **エンコーディング**: JPEG (HTTP) / sensor_msgs/Image (ROS2)

## ライセンス

MIT License

## 作成者

matsuda
