#!/usr/bin/env python3

import requests
import cv2
import numpy as np
import os
import tempfile
from urllib.parse import urlparse
import rclpy
from rclpy.node import Node

class ImageDownloader(Node):
    def __init__(self):
        super().__init__('image_downloader')
        
        # ヘッダーを設定（一部のサイトで必要）
        self.headers = {
            'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/91.0.4472.124 Safari/537.36'
        }
        
        self.get_logger().info('画像ダウンローダーが初期化されました')
    
    def download_image(self, url, max_size=(800, 600)):
        """
        指定されたURLから画像をダウンロードし、OpenCV形式で返す
        
        Args:
            url (str): 画像のURL
            max_size (tuple): 最大サイズ (width, height)
        
        Returns:
            numpy.ndarray: 画像データ（BGR形式）
        """
        try:
            self.get_logger().info(f'画像をダウンロード中: {url}')
            
            # リクエストを送信
            response = requests.get(url, headers=self.headers, timeout=30)
            response.raise_for_status()
            
            # 画像データを取得
            image_data = np.frombuffer(response.content, np.uint8)
            image = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
            
            if image is None:
                raise ValueError("画像のデコードに失敗しました")
            
            # サイズを調整
            height, width = image.shape[:2]
            if width > max_size[0] or height > max_size[1]:
                scale = min(max_size[0] / width, max_size[1] / height)
                new_width = int(width * scale)
                new_height = int(height * scale)
                image = cv2.resize(image, (new_width, new_height), interpolation=cv2.INTER_AREA)
            
            self.get_logger().info(f'画像ダウンロード完了: {image.shape}')
            return image
            
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f'ダウンロードエラー: {e}')
            return None
        except Exception as e:
            self.get_logger().error(f'画像処理エラー: {e}')
            return None
    
    def save_image(self, image, filename):
        """画像をファイルに保存"""
        try:
            cv2.imwrite(filename, image)
            self.get_logger().info(f'画像を保存しました: {filename}')
            return True
        except Exception as e:
            self.get_logger().error(f'保存エラー: {e}')
            return False
    
    def download_from_unsplash(self, query="cat", width=400, height=400):
        """
        Unsplashから画像をダウンロード（サンプル用）
        注意: 実際のAPIキーが必要です
        """
        # サンプル画像のURL（実際のプロジェクトでは適切なAPIを使用）
        sample_urls = [
            "https://images.unsplash.com/photo-1514888286974-6c03e2ca1dba?w=400&h=400&fit=crop",  # 猫
            "https://images.unsplash.com/photo-1552053831-71594a27632d?w=400&h=400&fit=crop",  # 犬
            "https://images.unsplash.com/photo-1506905925346-21bda4d32df4?w=400&h=400&fit=crop",  # 山
            "https://images.unsplash.com/photo-1506905925346-21bda4d32df4?w=400&h=400&fit=crop",  # 海
        ]
        
        # ランダムに選択
        import random
        url = random.choice(sample_urls)
        return self.download_image(url, (width, height))
    
    def download_sample_image(self):
        """サンプル画像を生成（ネット接続不要）"""
        # シンプルな図形を生成
        image = np.ones((400, 400, 3), dtype=np.uint8) * 255  # 白背景
        
        # 円を描画
        cv2.circle(image, (200, 200), 100, (0, 0, 0), 2)
        
        # 四角形を描画
        cv2.rectangle(image, (50, 50), (150, 150), (0, 0, 0), 2)
        
        # 三角形を描画
        triangle = np.array([[300, 100], [250, 200], [350, 200]], np.int32)
        cv2.polylines(image, [triangle], True, (0, 0, 0), 2)
        
        # 線を描画
        cv2.line(image, (100, 300), (300, 300), (0, 0, 0), 2)
        cv2.line(image, (200, 250), (200, 350), (0, 0, 0), 2)
        
        self.get_logger().info('サンプル画像を生成しました')
        return image

def main():
    rclpy.init()
    
    downloader = ImageDownloader()
    
    try:
        # サンプル画像を生成
        image = downloader.download_sample_image()
        
        if image is not None:
            # 画像を保存
            downloader.save_image(image, '/tmp/sample_image.jpg')
            
            # 画像を表示（X11フォワーディングが有効な場合）
            try:
                cv2.imshow('Downloaded Image', image)
                cv2.waitKey(0)
                cv2.destroyAllWindows()
            except:
                downloader.get_logger().info('画像表示をスキップしました（X11フォワーディングなし）')
        
    except Exception as e:
        downloader.get_logger().error(f'エラー: {e}')
    finally:
        downloader.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
