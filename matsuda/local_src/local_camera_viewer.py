#!/usr/bin/env python3
"""
ローカルPC用のカメラ映像直接表示ビューアー
camera_sender.pyの生のカメラ映像を表示
"""

import cv2
import requests
import time
import logging

# ログ設定
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class LocalCameraViewer:
    def __init__(self, stream_url="http://127.0.0.1:5001/frame"):
        self.stream_url = stream_url
        self.is_receiving = False
        self.frame_count = 0
        self.start_time = time.time()
        
    def start_receiving(self):
        """カメラ映像受信開始"""
        self.is_receiving = True
        self.start_time = time.time()
        self.frame_count = 0
        
        logger.info(f"ローカルカメラ映像受信開始: {self.stream_url}")
        
        try:
            while self.is_receiving:
                try:
                    # 単体の画像を取得
                    response = requests.get(self.stream_url, timeout=2)
                    response.raise_for_status()
                    
                    # JPEGデータを直接デコード
                    import numpy as np
                    nparr = np.frombuffer(response.content, np.uint8)
                    frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                    
                    if frame is not None:
                        # 統計情報を描画
                        self.draw_stats(frame)
                        
                        # フレーム表示
                        cv2.imshow("Local Camera (Raw)", frame)
                        
                        self.frame_count += 1
                        
                        # ESCキーで終了
                        key = cv2.waitKey(33) & 0xFF  # 30FPS
                        if key == 27:  # ESC
                            logger.info("ESCキーが押されました")
                            break
                    else:
                        logger.warning("フレームデコード失敗")
                        
                except requests.exceptions.RequestException as e:
                    logger.warning(f"画像取得エラー: {e}")
                    time.sleep(0.1)  # エラー時は少し待機
                except Exception as e:
                    logger.warning(f"フレーム処理エラー: {e}")
                    time.sleep(0.1)
                    
        except KeyboardInterrupt:
            logger.info("ユーザーによって停止されました")
        finally:
            logger.info("ローカルカメラ映像受信終了")
            cv2.destroyAllWindows()
    
    def draw_stats(self, frame):
        """統計情報をフレームに描画"""
        try:
            elapsed_time = time.time() - self.start_time
            fps = self.frame_count / elapsed_time if elapsed_time > 0 else 0
            
            stats_text = f"Local Camera FPS: {fps:.1f} | Frames: {self.frame_count} | Time: {elapsed_time:.1f}s"
            cv2.putText(frame, stats_text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # モード表示
            cv2.putText(frame, "RAW CAMERA MODE", (10, 60),
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
    logger.info("ローカルカメラ映像ビューアーを開始します...")
    
    # カメラビューアーを作成
    viewer = LocalCameraViewer()
    
    try:
        # 受信開始
        viewer.start_receiving()
    except Exception as e:
        logger.error(f"メイン処理でエラーが発生: {e}")
        import traceback
        logger.error(traceback.format_exc())
    finally:
        logger.info("ローカルカメラ映像ビューアー終了")

if __name__ == '__main__':
    main()
