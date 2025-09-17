#!/usr/bin/env python3
"""
ローカルカメラ画像送信サーバー
資料の12番に対応：ローカルカメラ画像をSSHトンネル経由で送信

使用方法:
1. このスクリプトをローカルで実行
2. SSHトンネルを設定: ssh -N -T -R 5009:127.0.0.1:5001 junkers12
3. リモートサーバーから http://127.0.0.1:5009/video でアクセス可能
"""

import cv2
import numpy as np
from flask import Flask, Response, jsonify
import threading
import time
import logging
import socket

# ログ設定
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

app = Flask(__name__)

class CameraStreamer:
    def __init__(self, camera_index=0):
        self.camera_index = camera_index
        self.cap = None
        self.is_streaming = False
        self.frame = None
        self.lock = threading.Lock()
        self.fps = 10  # テスト画像モードでは10FPSで十分
        self.frame_count = 0
        self.start_time = time.time()
        
    def initialize_camera(self):
        """カメラを初期化"""
        try:
            # タイムアウト付きでカメラを初期化
            logger.info(f"カメラ {self.camera_index} の初期化を試行中...")
            self.cap = cv2.VideoCapture(self.camera_index)
            
            # タイムアウト処理（5秒）
            start_time = time.time()
            while not self.cap.isOpened() and (time.time() - start_time) < 5.0:
                time.sleep(0.1)
            
            if not self.cap.isOpened():
                logger.warning(f"カメラ {self.camera_index} を開けませんでした。テスト画像モードで動作します。")
                if self.cap:
                    self.cap.release()
                self.cap = None
                return True
                
            # カメラ設定
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.cap.set(cv2.CAP_PROP_FPS, self.fps)
            
            # 実際の設定値を確認
            width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
            
            # カメラが正常に動作しているかチェック
            if width == 0 or height == 0:
                logger.warning(f"カメラの解像度が無効です ({width}x{height})。テスト画像モードで動作します。")
                if self.cap:
                    self.cap.release()
                self.cap = None
                return True
            
            logger.info(f"カメラ初期化成功: {width}x{height}, FPS: {actual_fps}")
            return True
            
        except Exception as e:
            logger.warning(f"カメラ初期化エラー: {e}。テスト画像モードで動作します。")
            if self.cap:
                self.cap.release()
            self.cap = None
            return True
    
    def start_streaming(self):
        """ストリーミング開始"""
        if self.is_streaming:
            return
            
        if not self.initialize_camera():
            logger.error("カメラ初期化に失敗しました")
            return
            
        self.is_streaming = True
        self.start_time = time.time()
        
        # 別スレッドでフレーム取得
        capture_thread = threading.Thread(target=self._capture_frames, daemon=True)
        capture_thread.start()
        logger.info("カメラストリーミング開始")
    
    def stop_streaming(self):
        """ストリーミング停止"""
        self.is_streaming = False
        if self.cap:
            self.cap.release()
        logger.info("カメラストリーミング停止")
    
    def _capture_frames(self):
        """フレーム取得ループ"""
        while self.is_streaming:
            try:
                if self.cap and self.cap.isOpened():
                    ret, frame = self.cap.read()
                    if ret:
                        with self.lock:
                            self.frame = frame.copy()
                            self.frame_count += 1
                    else:
                        logger.warning("カメラフレーム取得失敗")
                        time.sleep(0.1)
                        continue
                else:
                    # カメラが利用できない場合はテスト画像を生成
                    frame = self._generate_test_frame()
                    if frame is not None:
                        with self.lock:
                            self.frame = frame.copy()
                            self.frame_count += 1
                        
            except Exception as e:
                logger.error(f"フレーム取得中にエラー: {e}")
                time.sleep(0.1)
                continue
                
            time.sleep(0.1)  # 固定の短い待機時間
    
    def get_frame(self):
        """最新フレームを取得"""
        with self.lock:
            return self.frame.copy() if self.frame is not None else None
    
    def _generate_test_frame(self):
        """テスト用の画像フレームを生成（シンプル版）"""
        try:
            # 直接640x480で生成
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
            
            # 背景色を設定（明るいグレー）
            frame[:] = (100, 100, 100)
            
            # シンプルな図形を描画（テキストなし）
            # 移動する大きな円
            center_x = int(320 + 150 * np.sin(time.time() * 2))
            center_y = int(240 + 80 * np.cos(time.time() * 2))
            cv2.circle(frame, (center_x, center_y), 60, (0, 0, 255), -1)
            
            # 固定の小さい円
            cv2.circle(frame, (100, 100), 30, (0, 255, 0), -1)
            cv2.circle(frame, (540, 100), 30, (255, 0, 0), -1)
            cv2.circle(frame, (100, 380), 30, (255, 255, 0), -1)
            cv2.circle(frame, (540, 380), 30, (255, 0, 255), -1)
            
            # 四角形を描画
            cv2.rectangle(frame, (50, 200), (150, 280), (0, 255, 255), -1)
            cv2.rectangle(frame, (490, 200), (590, 280), (255, 255, 0), -1)
            
            # 枠線
            cv2.rectangle(frame, (10, 10), (630, 470), (255, 255, 255), 5)
            
            return frame
            
        except Exception as e:
            logger.error(f"テスト画像生成エラー: {e}")
            import traceback
            logger.error(traceback.format_exc())
            return None
    
    def get_stats(self):
        """統計情報を取得"""
        elapsed_time = time.time() - self.start_time
        current_fps = self.frame_count / elapsed_time if elapsed_time > 0 else 0
        
        return {
            "frame_count": self.frame_count,
            "elapsed_time": elapsed_time,
            "current_fps": current_fps,
            "is_streaming": self.is_streaming,
            "camera_available": self.cap is not None and self.cap.isOpened()
        }

# グローバルカメラストリーマー
camera_streamer = CameraStreamer()

def generate_frames():
    """フレーム生成ジェネレータ"""
    while True:
        try:
            frame = camera_streamer.get_frame()
            if frame is not None:
                # フレームをJPEGエンコード（品質を下げて転送速度向上）
                ret, buffer = cv2.imencode('.jpg', frame, 
                                         [cv2.IMWRITE_JPEG_QUALITY, 60])
                if ret:
                    frame_bytes = buffer.tobytes()
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + 
                           frame_bytes + b'\r\n')
            else:
                # フレームが取得できない場合は短い待機
                time.sleep(0.05)
                
        except Exception as e:
            logger.error(f"フレーム生成中にエラー: {e}")
            time.sleep(0.1)

@app.route('/')
def index():
    """メインページ"""
    stats = camera_streamer.get_stats()
    return f"""
    <html>
    <head><title>ローカルカメラ画像送信サーバー</title></head>
    <body>
        <h1>ローカルカメラ画像送信サーバー</h1>
        <p><strong>状態:</strong> {'ストリーミング中' if stats['is_streaming'] else '停止中'}</p>
        <p><strong>カメラ状態:</strong> {'利用可能' if stats['camera_available'] else 'テスト画像モード'}</p>
        <p><strong>フレーム数:</strong> {stats['frame_count']}</p>
        <p><strong>経過時間:</strong> {stats['elapsed_time']:.1f}秒</p>
        <p><strong>現在のFPS:</strong> {stats['current_fps']:.1f}</p>
        <p><a href="/video">映像ストリーム</a></p>
        <p><a href="/stats">統計情報 (JSON)</a></p>
    </body>
    </html>
    """

@app.route('/video')
def video():
    """映像ストリームエンドポイント"""
    return Response(generate_frames(),
                   mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/frame')
def get_frame():
    """単体フレーム取得エンドポイント"""
    try:
        frame = camera_streamer.get_frame()
        if frame is not None:
            ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ret:
                return Response(buffer.tobytes(), mimetype='image/jpeg')
        
        # フレームが取得できない場合はエラー
        return "No frame available", 404
    except Exception as e:
        logger.error(f"フレーム取得エラー: {e}")
        return "Error getting frame", 500

@app.route('/stats')
def stats():
    """統計情報エンドポイント"""
    return jsonify(camera_streamer.get_stats())

@app.route('/start')
def start_streaming():
    """ストリーミング開始"""
    camera_streamer.start_streaming()
    return jsonify({"status": "started"})

@app.route('/stop')
def stop_streaming():
    """ストリーミング停止"""
    camera_streamer.stop_streaming()
    return jsonify({"status": "stopped"})

def get_fixed_port():
    """固定ポートを取得（5001を優先、使用中の場合は警告）"""
    preferred_port = 5001
    
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind(('127.0.0.1', preferred_port))
            logger.info(f"固定ポート {preferred_port} が利用可能です")
            return preferred_port
    except OSError:
        logger.warning(f"固定ポート {preferred_port} は使用中です。代替ポートを検索します...")
        # 代替ポートを検索
        for port in range(preferred_port + 1, preferred_port + 6):
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    s.bind(('127.0.0.1', port))
                    logger.warning(f"代替ポート {port} を使用します")
                    return port
            except OSError:
                continue
        logger.error("利用可能なポートが見つかりません")
        return None

def main():
    """メイン関数"""
    logger.info("ローカルカメラ画像送信サーバーを開始します...")
    
    try:
        # カメラストリーミングを自動開始
        camera_streamer.start_streaming()
        
        # 固定ポートを取得
        port = get_fixed_port()
        if port is None:
            logger.error("利用可能なポートが見つかりません")
            return
        
        # Flaskサーバーを起動
        logger.info(f"サーバーを http://127.0.0.1:{port} で起動中...")
        
        if port == 5001:
            logger.info("=== 固定ポート設定 ===")
            logger.info("SSHトンネル設定: ssh -N -T -R 5009:127.0.0.1:5001 12")
            logger.info("リモートからアクセス: http://127.0.0.1:5009/frame")
        else:
            logger.warning("=== 代替ポート設定 ===")
            logger.warning(f"SSHトンネル設定: ssh -N -T -R 5009:127.0.0.1:{port} 12")
            logger.warning(f"リモートからアクセス: http://127.0.0.1:5009/frame")
        
        app.run(host='127.0.0.1', port=port, debug=False, threaded=True)
        
    except KeyboardInterrupt:
        logger.info("サーバーを停止中...")
    except Exception as e:
        logger.error(f"メイン処理でエラーが発生: {e}")
        import traceback
        logger.error(traceback.format_exc())
    finally:
        camera_streamer.stop_streaming()
        logger.info("サーバー終了完了")

if __name__ == '__main__':
    main()
