# SSH & トンネル完全ガイド : 11/12/13 サーバーと画像・音声送受信

このドキュメントは、ローカルPCから junkers11/12/13 にSSHで接続し、
**画像や音声をトンネル経由で送受信して、さらにROS2 Jazzyと統合したシステムを作る**ための手順をまとめています。
これだけ読めば、誰でも同じ仕組みを再現し、新しいシステムを開発できます。

---

## 1. 前提条件
* ローカルPCは Linux / macOS
* `ssh` コマンドが利用可能
* 管理者から渡された秘密鍵 `~/.ssh/id_ed25519.blitz`
* サーバ **11 / 12 / 13** には **ROS2 Jazzy** がインストール済み  
  (`source /opt/ros/jazzy/setup.bash` で利用可能)

---

## 2. SSH接続準備

### 2.1 ssh-agentに鍵を登録（セッション最初だけ）
```bash
eval "$(ssh-agent -s)"
ssh-add ~/.ssh/id_ed25519.blitz
```
パスフレーズを入力し、`ssh-add -l` で鍵が表示されればOK。

### 2.2 直接ジャンプして各サーバへ接続
blitz → junkers1 → 各サーバを一気に抜けます:
```bash
ssh -A -J matsuda@blitz.xack.co.jp,matsuda@junkers1 matsuda@junkers11
ssh -A -J matsuda@blitz.xack.co.jp,matsuda@junkers1 matsuda@junkers12
ssh -A -J matsuda@blitz.xack.co.jp,matsuda@junkers1 matsuda@junkers13
```
`-A` はエージェント転送を有効化します。

---

## 3. SSHトンネルでデータ転送

### ローカル→サーバへ送信 (RemoteForward)
ローカルのFlaskサーバ(127.0.0.1:5001)をサーバから見えるようにする:
```bash
ssh -N -T -R 5009:127.0.0.1:5001 11
```
→ 11から `http://127.0.0.1:5009` にアクセスするとローカル配信を取得可能。

12,13でも同様に:
```bash
ssh -N -T -R 5009:127.0.0.1:5001 12
ssh -N -T -R 5009:127.0.0.1:5001 13
```

### サーバ→ローカルへ戻す (LocalForward)
サーバで処理した映像/音声をローカルで見る:
```bash
ssh -N -T -L 7000:127.0.0.1:5002 11
ssh -N -T -L 7001:127.0.0.1:5002 12
ssh -N -T -L 7002:127.0.0.1:5002 13
```
→ ローカルブラウザで `http://127.0.0.1:7000` (11),
   `http://127.0.0.1:7001` (12),
   `http://127.0.0.1:7002` (13) にアクセスするとそれぞれの結果を表示。

---

## 4. 画像ストリーミング例

### ローカル送信サーバ
```python
from flask import Flask, Response
import cv2

app = Flask(__name__)
cap = cv2.VideoCapture(0)

def gen():
    while True:
        ret, frame = cap.read()
        if not ret: continue
        _, jpg = cv2.imencode('.jpg', frame)
        yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpg.tobytes() + b'\r\n'

@app.route('/video')
def video():
    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')

app.run('127.0.0.1', 5001)
```

### サーバ側受信
```python
import requests, cv2, numpy as np

url = "http://127.0.0.1:5009/video"
stream = requests.get(url, stream=True)
buf = b''
for chunk in stream.iter_content(1024):
    buf += chunk
    a = buf.find(b'\xff\xd8'); b = buf.find(b'\xff\xd9')
    if a != -1 and b != -1:
        jpg = buf[a:b+2]; buf = buf[b+2:]
        img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
        cv2.imshow("recv", img)
        if cv2.waitKey(1) == 27: break
```
→ 受信した映像を11/12/13で処理後、LocalForwardでローカルに戻してブラウザやOpenCVで表示可能。

---

## 5. 音声ストリーミング例

### ローカル送信
```python
import sounddevice as sd
from flask import Flask, Response
import numpy as np

app = Flask(__name__)

def audio_stream():
    while True:
        data = sd.rec(1024, samplerate=16000, channels=1, dtype='int16')
        sd.wait()
        yield data.tobytes()

@app.route('/audio')
def audio():
    return Response(audio_stream(), mimetype='audio/L16;rate=16000;channels=1')

app.run('127.0.0.1', 5001)
```
→ `http://127.0.0.1:5009/audio` で 11/12/13 が取得可能。

---

## 6. ROS2 Jazzy連携

- サーバ側で ROS2 を有効化:
```bash
source /opt/ros/jazzy/setup.bash
```
- 受信した画像を `sensor_msgs/msg/Image` などで publish/subscribe 可能。
- `audio_common_msgs/msg/AudioData` を使えば音声もTopicとして流せる。

---

## 7. セキュリティ注意
- トンネルは全て `127.0.0.1` バインドで外部公開しない。
- `.ssh` は `chmod 700 ~/.ssh`、鍵ファイルは `chmod 600`。
- 長期運用は公開鍵認証のみ。

---

## まとめ

1. ssh-agent に鍵を登録
2. `ssh -A -J blitz.xack.co.jp,junkers1 matsuda@junkers11` などで 11/12/13 に接続
3. SSH トンネルを使ってローカル⇄サーバ間で画像・音声をリアルタイム送受信
4. ROS2 Jazzy Topicに統合して自由に拡張

この手順を使えば、新しい画像・音声解析システムを自由に開発できます。
