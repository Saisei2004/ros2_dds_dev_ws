#!/usr/bin/env python3
"""
ローカルPC用のROS2画像表示クライアント
SSHトンネル経由でROS2画像を受信して表示
"""

import cv2
import numpy as np
import requests
import time
import logging

# ログ設定
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class LocalROS2ImageViewer:
    def __init__(self, stream_url="http://127.0.0.1:5010/video"):
        self.stream_url = stream_url
        self.is_receiving = False
        self.frame_count = 0
        self.start_time = time.time()
        
    def start_receiving(self):
        """ROS2画像受信開始"""
        self.is_receiving = True
        self.start_time = time.time()
        self.frame_count = 0
        
        logger.info(f"ローカルROS2画像受信開始: {self.stream_url}")
        
        try:
            # HTTPストリームに接続
            response = requests.get(self.stream_url, stream=True, timeout=5)
            response.raise_for_status()
            
            logger.info("ストリーム接続成功")
            
            buffer = b''
            for chunk in response.iter_content(chunk_size=8192):
                if not self.is_receiving:
                    break
                
                buffer += chunk
                
                # MJPEGストリームの境界を検索
                start_marker = buffer.find(b'\xff\xd8')  # JPEG開始
                end_marker = buffer.find(b'\xff\xd9')    # JPEG終了
                
                if start_marker != -1 and end_marker != -1:
                    jpeg_data = buffer[start_marker:end_marker + 2]
                    buffer = buffer[end_marker + 2:]
                    
                    try:
                        nparr = np.frombuffer(jpeg_data, np.uint8)
                        frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                        
                        if frame is not None:
                            self.frame_count += 1
                            self._display_frame(frame)
                            
                            key = cv2.waitKey(1) & 0xFF
                            if key == 27:  # ESC
                                logger.info("ESCキーが押されました")
                                break
                        else:
                            logger.warning("フレームデコード失敗")
                            
                    except Exception as e:
                        logger.warning(f"フレーム処理エラー: {e}")
                        
        except requests.exceptions.RequestException as e:
            logger.error(f"ストリーム接続エラー: {e}")
        except Exception as e:
            logger.error(f"予期しないエラー: {e}")
        finally:
            logger.info("ストリーム受信終了")
            cv2.destroyAllWindows()
    
    
    def _display_frame(self, frame):
        """フレームを表示"""
        try:
            # 統計情報を描画
            self._draw_stats(frame)
            
            # フレーム表示
            cv2.imshow("Local ROS2 Image Viewer", frame)
            
        except Exception as e:
            logger.error(f"フレーム表示エラー: {e}")
    
    def _draw_stats(self, frame):
        """統計情報をフレームに描画"""
        try:
            # FPS計算
            elapsed = time.time() - self.start_time
            fps = self.frame_count / elapsed if elapsed > 0 else 0
            
            # 統計情報テキスト
            stats_text = f"ROS2 Images: {self.frame_count} | FPS: {fps:.1f}"
            
            # テキストを描画
            cv2.putText(frame, stats_text, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # 接続状態を表示
            cv2.putText(frame, "SSH Tunnel: Connected", (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            # 操作説明
            cv2.putText(frame, "ESC: Exit", (10, frame.shape[0] - 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
        except Exception as e:
            logger.error(f"統計情報描画エラー: {e}")
    
    def stop_receiving(self):
        """受信停止"""
        self.is_receiving = False

def main():
    """メイン関数"""
    logger.info("ローカルROS2画像ビューアーを開始します...")
    
    # 画像ビューアーを作成
    viewer = LocalROS2ImageViewer()
    
    try:
        # 受信開始
        viewer.start_receiving()
    except Exception as e:
        logger.error(f"メイン処理でエラーが発生: {e}")
        import traceback
        logger.error(traceback.format_exc())
    finally:
        logger.info("ローカルROS2画像ビューアー終了")

if __name__ == '__main__':
    main()
