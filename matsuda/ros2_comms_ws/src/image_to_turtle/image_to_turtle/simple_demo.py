#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import time
import subprocess
import os
from image_to_turtle.image_downloader import ImageDownloader
from image_to_turtle.line_detector import LineDetector
from image_to_turtle.turtle_drawer import TurtleDrawer

class SimpleDemo(Node):
    def __init__(self):
        super().__init__('simple_demo')
        
        # 各コンポーネントを初期化
        self.downloader = ImageDownloader()
        self.detector = LineDetector()
        self.drawer = TurtleDrawer()
        
        self.get_logger().info('シンプルデモが初期化されました')
    
    def display_image(self, image_path, title="画像"):
        """画像を表示（SSH対応版）"""
        try:
            if os.path.exists(image_path):
                self.get_logger().info(f'=== {title} を表示中 ===')
                self.get_logger().info(f'画像パス: {image_path}')
                
                # SSH環境でも動作するように複数の方法を試行
                display_commands = [
                    ['display', image_path, '-title', title],
                    ['feh', image_path, '--title', title],
                    ['eog', image_path],
                    ['xdg-open', image_path],
                    ['gpicview', image_path],
                    ['gwenview', image_path]
                ]
                
                displayed = False
                for cmd in display_commands:
                    try:
                        self.get_logger().info(f'試行中: {" ".join(cmd)}')
                        process = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                        self.get_logger().info(f'✅ {title}を表示しました: {image_path} (コマンド: {" ".join(cmd)})')
                        displayed = True
                        break
                    except FileNotFoundError:
                        self.get_logger().debug(f'コマンドが見つかりません: {" ".join(cmd)}')
                        continue
                
                if not displayed:
                    self.get_logger().warn(f'❌ 画像表示コマンドが見つかりません')
                    self.get_logger().info(f'画像ファイルの場所: {image_path}')
                    self.get_logger().info('手動で画像を確認してください')
                
                time.sleep(2)  # 表示のための待機
            else:
                self.get_logger().warn(f'❌ 画像ファイルが見つかりません: {image_path}')
        except Exception as e:
            self.get_logger().error(f'❌ 画像表示に失敗しました: {e}')
            self.get_logger().info(f'画像ファイルの場所: {image_path}')
    
    def save_debug_images(self, original_image):
        """デバッグ用画像を保存"""
        try:
            import os
            
            debug_dir = "/tmp/turtle_debug"
            os.makedirs(debug_dir, exist_ok=True)
            
            # 元画像
            cv2.imwrite(f"{debug_dir}/01_original.png", original_image)
            self.get_logger().info(f'元画像を保存: {debug_dir}/01_original.png')
            
            # 前処理後画像
            processed = self.detector.preprocess_image(original_image)
            cv2.imwrite(f"{debug_dir}/02_processed.png", processed)
            self.get_logger().info(f'前処理後画像を保存: {debug_dir}/02_processed.png')
            
            # エッジ検出結果
            edges = self.detector.extract_edges(processed)
            cv2.imwrite(f"{debug_dir}/03_edges.png", edges)
            self.get_logger().info(f'エッジ検出結果を保存: {debug_dir}/03_edges.png')
            
            # Hough線検出結果
            hough_lines = self.detector.detect_lines_hough(edges)
            if len(hough_lines) > 0:
                hough_vis = original_image.copy()
                for line in hough_lines:
                    x1, y1, x2, y2 = line
                    cv2.line(hough_vis, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.imwrite(f"{debug_dir}/04_hough_lines.png", hough_vis)
                self.get_logger().info(f'Hough線検出結果を保存: {debug_dir}/04_hough_lines.png ({len(hough_lines)}本)')
            
            # 輪郭検出結果
            contour_lines = self.detector.detect_contours(edges)
            if len(contour_lines) > 0:
                contour_vis = original_image.copy()
                for line in contour_lines:
                    x1, y1, x2, y2 = line
                    cv2.line(contour_vis, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 2)
                cv2.imwrite(f"{debug_dir}/05_contour_lines.png", contour_vis)
                self.get_logger().info(f'輪郭線検出結果を保存: {debug_dir}/05_contour_lines.png ({len(contour_lines)}本)')
            
            return debug_dir
            
        except Exception as e:
            self.get_logger().error(f'デバッグ画像保存エラー: {e}')
            return None
    
    def print_line_debug_info(self, lines):
        """線のデバッグ情報を表示"""
        self.get_logger().info('=== 線検出デバッグ情報 ===')
        self.get_logger().info(f'検出された線の数: {len(lines)}')
        
        # 角度の分布を確認
        angles = []
        for i, line in enumerate(lines[:20]):  # 最初の20本を表示
            x1, y1, x2, y2 = line
            length = np.sqrt((x2-x1)**2 + (y2-y1)**2)
            angle = np.degrees(np.arctan2(y2-y1, x2-x1))
            angles.append(angle)
            self.get_logger().info(f'線{i+1}: ({x1:.2f},{y1:.2f})->({x2:.2f},{y2:.2f}) 長さ:{length:.2f} 角度:{angle:.1f}度')
        
        if len(lines) > 20:
            self.get_logger().info(f'... 他{len(lines)-20}本の線')
        
        # 角度の統計情報
        if angles:
            angles = np.array(angles)
            self.get_logger().info(f'角度統計: 最小:{angles.min():.1f}度, 最大:{angles.max():.1f}度, 平均:{angles.mean():.1f}度')
        
        self.get_logger().info('=== デバッグ情報終了 ===')
    
    def display_debug_images(self, debug_dir):
        """デバッグ画像を順番に表示"""
        self.get_logger().info('=== デバッグ画像を表示します ===')
        
        debug_images = [
            ('01_original.png', '元画像'),
            ('02_processed.png', '前処理後'),
            ('03_edges.png', 'エッジ検出結果'),
            ('04_hough_lines.png', 'Hough線検出結果'),
            ('05_contour_lines.png', '輪郭線検出結果')
        ]
        
        for filename, title in debug_images:
            image_path = f"{debug_dir}/{filename}"
            if os.path.exists(image_path):
                self.get_logger().info(f'表示中: {title}')
                self.display_image(image_path, title)
                time.sleep(3)  # 各画像を3秒表示
            else:
                self.get_logger().warn(f'画像が見つかりません: {image_path}')
        
        self.get_logger().info('=== デバッグ画像表示完了 ===')

    def run_demo(self):
        """デモを実行"""
        self.get_logger().info('=== Image to Turtle デモ開始 ===')
        
        try:
            # 1. サンプル画像を生成
            self.get_logger().info('ステップ1: サンプル画像生成')
            image = self.downloader.download_sample_image()
            
            if image is None:
                self.get_logger().error('サンプル画像の生成に失敗しました')
                return False
            
            # 画像を保存
            self.downloader.save_image(image, '/tmp/demo_sample.jpg')
            self.get_logger().info('サンプル画像を保存しました: /tmp/demo_sample.jpg')
            
            # 元の画像を表示
            self.display_image('/tmp/demo_sample.jpg', '元の画像 - これから描画します')
            
            # 2. 線を検出（デバッグ版）
            self.get_logger().info('ステップ2: 線検出（デバッグ版）')
            
            # デバッグ用：各段階の画像を保存・表示
            debug_dir = self.save_debug_images(image)
            if debug_dir:
                self.display_debug_images(debug_dir)
            
            lines = self.detector.process_image(image)
            
            if len(lines) == 0:
                self.get_logger().warn('線が検出されませんでした')
                return False
            
            self.get_logger().info(f'{len(lines)}本の線を検出しました')
            
            # 線検出結果を可視化
            vis_image = self.detector.visualize_lines(image, lines, '/tmp/demo_detected_lines.jpg')
            self.get_logger().info('線検出結果を保存しました: /tmp/demo_detected_lines.jpg')
            
            # 検出された線を表示
            self.display_image('/tmp/demo_detected_lines.jpg', '検出された線 - タートルが描画する内容')
            
            # デバッグ情報を表示
            self.print_line_debug_info(lines)
            
            # 3. タートルで描画
            self.get_logger().info('ステップ3: タートル描画開始')
            self.get_logger().info('タートルシムウィンドウで描画を確認してください！')
            self.drawer.draw_lines(lines)
            
            self.get_logger().info('=== デモ完了 ===')
            return True
            
        except Exception as e:
            self.get_logger().error(f'デモ実行中にエラーが発生しました: {e}')
            return False

def main(args=None):
    rclpy.init(args=args)
    
    demo = SimpleDemo()
    
    try:
        # デモを実行
        success = demo.run_demo()
        
        if success:
            demo.get_logger().info('デモが正常に完了しました！')
        else:
            demo.get_logger().error('デモが失敗しました')
        
        # 少し待機してから終了
        time.sleep(2)
        
    except Exception as e:
        demo.get_logger().error(f'エラー: {e}')
    finally:
        demo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
