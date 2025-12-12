#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import time
import os
import tempfile
from image_to_turtle.image_downloader import ImageDownloader
from image_to_turtle.line_detector import LineDetector
from image_to_turtle.turtle_drawer import TurtleDrawer

class ImageToTurtleMain(Node):
    def __init__(self):
        super().__init__('image_to_turtle_main')
        
        # 各コンポーネントを初期化
        self.downloader = ImageDownloader()
        self.detector = LineDetector()
        self.drawer = TurtleDrawer()
        
        self.get_logger().info('Image to Turtle メインノードが初期化されました')
    
    def process_image_from_url(self, url):
        """
        URLから画像を取得してタートルで描画
        """
        self.get_logger().info(f'URLから画像を処理開始: {url}')
        
        try:
            # 1. 画像をダウンロード
            self.get_logger().info('ステップ1: 画像ダウンロード')
            image = self.downloader.download_image(url)
            
            if image is None:
                self.get_logger().error('画像のダウンロードに失敗しました')
                return False
            
            # 2. 線を検出
            self.get_logger().info('ステップ2: 線検出')
            lines = self.detector.process_image(image)
            
            if len(lines) == 0:
                self.get_logger().warn('線が検出されませんでした')
                return False
            
            # 3. タートルで描画
            self.get_logger().info('ステップ3: タートル描画')
            self.drawer.draw_lines(lines)
            
            self.get_logger().info('画像の描画が完了しました')
            return True
            
        except Exception as e:
            self.get_logger().error(f'処理中にエラーが発生しました: {e}')
            return False
    
    def process_sample_image(self):
        """
        サンプル画像を生成してタートルで描画
        """
        self.get_logger().info('サンプル画像を処理開始')
        
        try:
            # 1. サンプル画像を生成
            self.get_logger().info('ステップ1: サンプル画像生成')
            image = self.downloader.download_sample_image()
            
            # 画像を保存（デバッグ用）
            self.downloader.save_image(image, '/tmp/generated_sample.jpg')
            
            # 2. 線を検出
            self.get_logger().info('ステップ2: 線検出')
            lines = self.detector.process_image(image)
            
            if len(lines) == 0:
                self.get_logger().warn('線が検出されませんでした')
                return False
            
            # 線検出結果を可視化
            vis_image = self.detector.visualize_lines(image, lines, '/tmp/detected_lines.jpg')
            
            # 3. タートルで描画
            self.get_logger().info('ステップ3: タートル描画')
            self.drawer.draw_lines(lines)
            
            self.get_logger().info('サンプル画像の描画が完了しました')
            return True
            
        except Exception as e:
            self.get_logger().error(f'処理中にエラーが発生しました: {e}')
            return False
    
    def process_image_file(self, image_path):
        """
        ファイルから画像を読み込んでタートルで描画
        """
        self.get_logger().info(f'ファイルから画像を処理開始: {image_path}')
        
        try:
            # 1. 画像を読み込み
            self.get_logger().info('ステップ1: 画像読み込み')
            image = cv2.imread(image_path)
            
            if image is None:
                self.get_logger().error(f'画像ファイルを読み込めませんでした: {image_path}')
                return False
            
            # 2. 線を検出
            self.get_logger().info('ステップ2: 線検出')
            lines = self.detector.process_image(image)
            
            if len(lines) == 0:
                self.get_logger().warn('線が検出されませんでした')
                return False
            
            # 線検出結果を可視化
            vis_image = self.detector.visualize_lines(image, lines, '/tmp/detected_lines.jpg')
            
            # 3. タートルで描画
            self.get_logger().info('ステップ3: タートル描画')
            self.drawer.draw_lines(lines)
            
            self.get_logger().info('画像ファイルの描画が完了しました')
            return True
            
        except Exception as e:
            self.get_logger().error(f'処理中にエラーが発生しました: {e}')
            return False
    
    def run_interactive_mode(self):
        """
        インタラクティブモードで実行
        """
        self.get_logger().info('インタラクティブモードを開始')
        self.get_logger().info('利用可能なコマンド:')
        self.get_logger().info('  1: サンプル画像を描画')
        self.get_logger().info('  2: 画像ファイルを描画')
        self.get_logger().info('  3: 画面をクリア')
        self.get_logger().info('  q: 終了')
        
        while rclpy.ok():
            try:
                command = input('\nコマンドを入力してください (1-3, q): ').strip()
                
                if command == '1':
                    self.process_sample_image()
                elif command == '2':
                    image_path = input('画像ファイルのパスを入力してください: ').strip()
                    if os.path.exists(image_path):
                        self.process_image_file(image_path)
                    else:
                        self.get_logger().error(f'ファイルが見つかりません: {image_path}')
                elif command == '3':
                    self.drawer.clear_screen()
                elif command.lower() == 'q':
                    self.get_logger().info('終了します')
                    break
                else:
                    self.get_logger().warn('無効なコマンドです')
                    
            except KeyboardInterrupt:
                self.get_logger().info('キーボード割り込みで終了')
                break
            except Exception as e:
                self.get_logger().error(f'エラー: {e}')

def main(args=None):
    rclpy.init(args=args)
    
    main_node = ImageToTurtleMain()
    
    try:
        # インタラクティブモードで実行
        main_node.run_interactive_mode()
        
    except Exception as e:
        main_node.get_logger().error(f'エラー: {e}')
    finally:
        main_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
