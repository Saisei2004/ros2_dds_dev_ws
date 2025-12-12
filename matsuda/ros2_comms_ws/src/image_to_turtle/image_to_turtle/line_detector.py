#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from typing import List, Tuple

class LineDetector(Node):
    def __init__(self):
        super().__init__('line_detector')
        
        self.get_logger().info('線検出器が初期化されました')
    
    def preprocess_image(self, image):
        """
        画像の前処理（高精度版）
        """
        # グレースケール変換
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image.copy()
        
        # ノイズ除去（バイラテラルフィルタ）
        denoised = cv2.bilateralFilter(gray, 9, 75, 75)
        
        # コントラスト強調（CLAHE）
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        enhanced = clahe.apply(denoised)
        
        # ガウシアンフィルタで軽くぼかし
        blurred = cv2.GaussianBlur(enhanced, (3, 3), 0)
        
        # 適応的閾値処理（複数の方法を試行）
        binary1 = cv2.adaptiveThreshold(
            blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2
        )
        
        binary2 = cv2.adaptiveThreshold(
            blurred, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 15, 3
        )
        
        # 2つの結果を合成
        binary = cv2.bitwise_and(binary1, binary2)
        
        # モルフォロジー演算でノイズ除去
        kernel = np.ones((2, 2), np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
        
        return binary
    
    def extract_edges(self, image):
        """
        エッジ検出（高精度版）
        """
        # 複数の閾値でエッジ検出
        edges1 = cv2.Canny(image, 30, 100, apertureSize=3)
        edges2 = cv2.Canny(image, 50, 150, apertureSize=3)
        edges3 = cv2.Canny(image, 80, 200, apertureSize=3)
        
        # 複数の結果を合成
        edges = cv2.bitwise_or(edges1, edges2)
        edges = cv2.bitwise_or(edges, edges3)
        
        # モルフォロジー演算で線を太くする
        kernel = np.ones((2, 2), np.uint8)
        edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
        
        # 細い線を太くする
        kernel_dilate = np.ones((1, 1), np.uint8)
        edges = cv2.dilate(edges, kernel_dilate, iterations=1)
        
        return edges
    
    def detect_lines_hough(self, edges):
        """
        Hough変換で直線を検出（正確版）
        """
        all_lines = []
        
        # パラメータセット1: 基本的な線検出（精度重視）
        lines1 = cv2.HoughLinesP(
            edges, 
            rho=1, 
            theta=np.pi/180, 
            threshold=30, 
            minLineLength=15, 
            maxLineGap=3
        )
        
        # パラメータセット2: 中程度の線検出
        lines2 = cv2.HoughLinesP(
            edges, 
            rho=1, 
            theta=np.pi/180, 
            threshold=50, 
            minLineLength=30, 
            maxLineGap=5
        )
        
        # パラメータセット3: 長い線検出
        lines3 = cv2.HoughLinesP(
            edges, 
            rho=2, 
            theta=np.pi/180, 
            threshold=80, 
            minLineLength=60, 
            maxLineGap=10
        )
        
        # すべての結果を結合
        for lines in [lines1, lines2, lines3]:
            if lines is not None:
                all_lines.extend(lines.reshape(-1, 4))
        
        return np.array(all_lines) if all_lines else np.array([])
    
    def detect_contours(self, edges):
        """
        輪郭検出
        """
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # 輪郭を線分に変換
        lines = []
        for contour in contours:
            if len(contour) > 1:
                # 輪郭を線分に分割
                for i in range(len(contour) - 1):
                    pt1 = contour[i][0]
                    pt2 = contour[i + 1][0]
                    lines.append([pt1[0], pt1[1], pt2[0], pt2[1]])
        
        return np.array(lines) if lines else np.array([])
    
    def simplify_lines(self, lines, tolerance=5):
        """
        線を簡素化（正確版）
        """
        if len(lines) == 0:
            return np.array([])
        
        # 重複する線を除去
        unique_lines = []
        for line in lines:
            x1, y1, x2, y2 = line
            # 線の長さを計算
            length = np.sqrt((x2-x1)**2 + (y2-y1)**2)
            if length > 3:  # 短すぎる線は除外
                unique_lines.append(line)
        
        if len(unique_lines) == 0:
            return np.array([])
        
        unique_lines = np.array(unique_lines)
        simplified = []
        used = set()
        
        for i, line1 in enumerate(unique_lines):
            if i in used:
                continue
                
            x1, y1, x2, y2 = line1
            current_line = [x1, y1, x2, y2]
            
            # 近い線を探して結合
            for j, line2 in enumerate(unique_lines[i+1:], i+1):
                if j in used:
                    continue
                    
                x3, y3, x4, y4 = line2
                
                # 線の端点が近いかチェック（距離ベース）
                dist1 = np.sqrt((x2 - x3)**2 + (y2 - y3)**2)
                dist2 = np.sqrt((x2 - x4)**2 + (y2 - y4)**2)
                dist3 = np.sqrt((x1 - x3)**2 + (y1 - y3)**2)
                dist4 = np.sqrt((x1 - x4)**2 + (y1 - y4)**2)
                
                if dist1 < tolerance:
                    # 線を結合
                    current_line = [x1, y1, x4, y4]
                    used.add(j)
                elif dist2 < tolerance:
                    current_line = [x1, y1, x3, y3]
                    used.add(j)
                elif dist3 < tolerance:
                    current_line = [x4, y4, x2, y2]
                    used.add(j)
                elif dist4 < tolerance:
                    current_line = [x3, y3, x2, y2]
                    used.add(j)
            
            simplified.append(current_line)
            used.add(i)
        
        return np.array(simplified)
    
    def convert_to_turtle_coordinates(self, lines, image_shape):
        """
        画像座標をタートルシム座標に変換（角度保持版）
        タートルシムの座標系: (0,0)が左下、x軸右向き、y軸上向き
        タートルシムの画面サイズ: 11x11 (0-11の範囲)
        """
        height, width = image_shape[:2]
        turtle_lines = []
        
        # 画像の中心点
        center_x = width / 2.0
        center_y = height / 2.0
        
        # タートルシムの中心点
        turtle_center_x = 5.5
        turtle_center_y = 5.5
        
        # スケールファクター（画像をタートルシムの画面に収める）
        # より安全な範囲を使用
        scale_x = 8.0 / width  # 横方向のスケール（1.0-9.0の範囲）
        scale_y = 8.0 / height  # 縦方向のスケール（1.0-9.0の範囲）
        
        # アスペクト比を保持するために、小さい方のスケールを使用
        scale = min(scale_x, scale_y)
        
        self.get_logger().info(f'画像サイズ: {width}x{height}, スケール: {scale:.4f}')
        
        for line in lines:
            x1, y1, x2, y2 = line
            
            # 元の角度を計算（デバッグ用）
            original_angle = np.degrees(np.arctan2(y2-y1, x2-x1))
            
            # 画像座標を中心を基準に変換
            # 画像の座標系: (0,0)が左上、x軸右向き、y軸下向き
            # タートルシムの座標系: (0,0)が左下、x軸右向き、y軸上向き
            
            # 1. 画像の中心を原点に移動
            rel_x1 = x1 - center_x
            rel_y1 = y1 - center_y
            rel_x2 = x2 - center_x
            rel_y2 = y2 - center_y
            
            # 2. スケール変換
            scaled_x1 = rel_x1 * scale
            scaled_y1 = rel_y1 * scale
            scaled_x2 = rel_x2 * scale
            scaled_y2 = rel_y2 * scale
            
            # 3. y軸を反転（画像のy軸は下向き、タートルシムのy軸は上向き）
            scaled_y1 = -scaled_y1
            scaled_y2 = -scaled_y2
            
            # 4. タートルシムの中心に移動
            tx1 = scaled_x1 + turtle_center_x
            ty1 = scaled_y1 + turtle_center_y
            tx2 = scaled_x2 + turtle_center_x
            ty2 = scaled_y2 + turtle_center_y
            
            # 5. 座標を画面内に制限（壁衝突を防ぐため安全マージンを設ける）
            tx1 = max(1.0, min(10.0, tx1))
            ty1 = max(1.0, min(10.0, ty1))
            tx2 = max(1.0, min(10.0, tx2))
            ty2 = max(1.0, min(10.0, ty2))
            
            # 6. 短すぎる線は除外
            distance = np.sqrt((tx2-tx1)**2 + (ty2-ty1)**2)
            if distance > 0.1:  # 最小距離
                # 変換後の角度を計算（デバッグ用）
                converted_angle = np.degrees(np.arctan2(ty2-ty1, tx2-tx1))
                
                turtle_lines.append([tx1, ty1, tx2, ty2])
                self.get_logger().info(f'角度変換: {original_angle:.1f}度 -> {converted_angle:.1f}度')
                self.get_logger().debug(f'座標変換: ({x1:.1f},{y1:.1f})->({x2:.1f},{y2:.1f}) -> ({tx1:.3f},{ty1:.3f})->({tx2:.3f},{ty2:.3f})')
        
        return np.array(turtle_lines)
    
    def sort_lines_by_drawing_order(self, lines):
        """
        線を描画順序でソート（近い線から順番に）
        """
        if len(lines) == 0:
            return lines
        
        sorted_lines = []
        remaining_lines = lines.tolist()
        current_pos = [5.5, 5.5]  # 開始位置
        
        while remaining_lines:
            # 現在位置から最も近い線を見つける
            min_distance = float('inf')
            closest_line_idx = 0
            
            for i, line in enumerate(remaining_lines):
                x1, y1, x2, y2 = line
                # 線の両端からの距離を計算
                dist1 = np.sqrt((x1 - current_pos[0])**2 + (y1 - current_pos[1])**2)
                dist2 = np.sqrt((x2 - current_pos[0])**2 + (y2 - current_pos[1])**2)
                min_dist = min(dist1, dist2)
                
                if min_dist < min_distance:
                    min_distance = min_dist
                    closest_line_idx = i
            
            # 最も近い線を選択
            closest_line = remaining_lines.pop(closest_line_idx)
            sorted_lines.append(closest_line)
            
            # 次の位置を更新（線の終点）
            x1, y1, x2, y2 = closest_line
            dist1 = np.sqrt((x1 - current_pos[0])**2 + (y1 - current_pos[1])**2)
            dist2 = np.sqrt((x2 - current_pos[0])**2 + (y2 - current_pos[1])**2)
            
            if dist1 < dist2:
                current_pos = [x2, y2]
            else:
                current_pos = [x1, y1]
        
        return np.array(sorted_lines)
    
    def process_image(self, image):
        """
        画像から線を抽出し、タートル座標に変換
        """
        self.get_logger().info('画像の線検出を開始...')
        
        # 前処理
        processed = self.preprocess_image(image)
        
        # エッジ検出
        edges = self.extract_edges(processed)
        
        # 線検出（Hough変換）
        hough_lines = self.detect_lines_hough(edges)
        
        # 輪郭検出
        contour_lines = self.detect_contours(edges)
        
        # 線を結合
        all_lines = []
        if len(hough_lines) > 0:
            all_lines.extend(hough_lines)
        if len(contour_lines) > 0:
            all_lines.extend(contour_lines)
        
        if len(all_lines) == 0:
            self.get_logger().warn('線が検出されませんでした')
            return np.array([])
        
        all_lines = np.array(all_lines)
        
        # 線を簡素化
        simplified_lines = self.simplify_lines(all_lines)
        
        # タートル座標に変換
        turtle_lines = self.convert_to_turtle_coordinates(simplified_lines, image.shape)
        
        # 線を描画順序でソート（近い線から順番に）
        if len(turtle_lines) > 0:
            turtle_lines = self.sort_lines_by_drawing_order(turtle_lines)
        
        self.get_logger().info(f'{len(turtle_lines)}本の線を検出しました')
        
        return turtle_lines
    
    def visualize_lines(self, image, lines, output_path=None):
        """
        検出した線を可視化
        """
        vis_image = image.copy()
        
        for line in lines:
            x1, y1, x2, y2 = line
            cv2.line(vis_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
        
        if output_path:
            cv2.imwrite(output_path, vis_image)
            self.get_logger().info(f'線検出結果を保存しました: {output_path}')
        
        return vis_image

def main():
    rclpy.init()
    
    detector = LineDetector()
    
    try:
        # サンプル画像を読み込み
        image = cv2.imread('/tmp/sample_image.jpg')
        if image is None:
            detector.get_logger().error('サンプル画像が見つかりません')
            return
        
        # 線検出
        lines = detector.process_image(image)
        
        if len(lines) > 0:
            # 結果を可視化
            vis_image = detector.visualize_lines(image, lines, '/tmp/detected_lines.jpg')
            
            # 画像を表示
            try:
                cv2.imshow('Original', image)
                cv2.imshow('Detected Lines', vis_image)
                cv2.waitKey(0)
                cv2.destroyAllWindows()
            except:
                detector.get_logger().info('画像表示をスキップしました')
        
    except Exception as e:
        detector.get_logger().error(f'エラー: {e}')
    finally:
        detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
