#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn, Kill, SetPen
from turtlesim.msg import Pose
import numpy as np
import time
import math
from typing import List, Tuple

class TurtleDrawer(Node):
    def __init__(self):
        super().__init__('turtle_drawer')
        
        # パブリッシャーとクライアントを作成
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.spawn_client = self.create_client(Spawn, '/spawn')
        self.kill_client = self.create_client(Kill, '/kill')
        self.set_pen_client = self.create_client(SetPen, '/turtle1/set_pen')
        
        # タートルの現在位置を購読
        self.pose_subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )
        
        # 現在の位置と向き（実際の位置で初期化）
        self.current_x = 5.5
        self.current_y = 5.5
        self.current_theta = 0.0
        self.pose_received = False
        
        # 描画設定（シンプル・確実版）
        self.pen_down = True
        self.drawing_speed = 2.0  # 適度な速度
        self.turning_speed = 2.0  # 適度な速度
        
        # 色設定
        self.colors = [
            (255, 0, 0),    # 赤
            (0, 255, 0),    # 緑
            (0, 0, 255),    # 青
            (255, 255, 0),  # 黄
            (255, 0, 255),  # マゼンタ
            (0, 255, 255),  # シアン
            (255, 128, 0),  # オレンジ
            (128, 0, 255),  # 紫
        ]
        self.current_color_index = 0
        
        self.get_logger().info('タートルドロワーが初期化されました')
        
        # サービスが利用可能になるまで待機
        self.wait_for_services()
    
    def pose_callback(self, msg):
        """タートルの現在位置を更新"""
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_theta = msg.theta
        self.pose_received = True
        self.get_logger().debug(f'現在位置: ({self.current_x:.3f}, {self.current_y:.3f}), 角度: {math.degrees(self.current_theta):.1f}度')
    
    def wait_for_services(self):
        """必要なサービスが利用可能になるまで待機"""
        self.get_logger().info('サービスを待機中...')
        
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('spawnサービスを待機中...')
        
        while not self.kill_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('killサービスを待機中...')
        
        while not self.set_pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('set_penサービスを待機中...')
        
        self.get_logger().info('すべてのサービスが利用可能になりました')
        
        # タートルの位置情報を待機
        self.wait_for_pose()
    
    def wait_for_pose(self):
        """タートルの位置情報を待機"""
        self.get_logger().info('タートルの位置情報を待機中...')
        
        timeout = 10.0  # 10秒でタイムアウト
        start_time = time.time()
        
        while not self.pose_received and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if self.pose_received:
            self.get_logger().info(f'タートルの初期位置を取得: ({self.current_x:.3f}, {self.current_y:.3f})')
        else:
            self.get_logger().warn('タートルの位置情報を取得できませんでした。デフォルト位置を使用します。')
    
    def set_pen(self, r=255, g=255, b=255, width=2, off=False):
        """ペンの設定"""
        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = width
        request.off = off
        
        future = self.set_pen_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.pen_down = not off
            self.get_logger().info(f'ペン設定: {"OFF" if off else "ON"}')
        else:
            self.get_logger().error('ペン設定に失敗しました')
    
    def move_to_position(self, target_x, target_y, draw=True):
        """
        指定された位置に移動（超精度版）
        
        Args:
            target_x (float): 目標X座標
            target_y (float): 目標Y座標
            draw (bool): 描画するかどうか
        """
        # 座標を安全範囲に制限
        target_x = max(1.0, min(10.0, target_x))
        target_y = max(1.0, min(10.0, target_y))
        
        # ペンの状態を設定
        if draw and not self.pen_down:
            self.set_pen(off=False)
        elif not draw and self.pen_down:
            self.set_pen(off=True)
        
        # 距離と角度を計算
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        distance = math.sqrt(dx*dx + dy*dy)
        
        # 距離が短すぎる場合は移動しない
        if distance < 0.01:
            return
        
        target_angle = math.atan2(dy, dx)
        
        # 回転（角度差が小さい場合は回転をスキップ）
        angle_diff = target_angle - self.current_theta
        # 角度を-πからπの範囲に正規化
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        # 角度差が大きい場合のみ回転
        if abs(angle_diff) > 0.02:  # 精度を上げるために閾値を下げる
            self.turn(angle_diff)
        
        # 直進
        self.move_forward(distance)
        
        # 実際の位置を取得して更新
        self.update_current_position()
    
    def update_current_position(self):
        """実際のタートルの位置を取得して更新"""
        # 少し待機してから位置を取得
        time.sleep(0.1)
        rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().debug(f'位置更新: ({self.current_x:.3f}, {self.current_y:.3f})')
    
    def turn(self, angle):
        """指定された角度だけ回転（超精度版）"""
        if abs(angle) < 0.001:  # 非常に小さい角度は無視
            return
            
        twist = Twist()
        twist.angular.z = self.turning_speed if angle > 0 else -self.turning_speed
        
        # 回転時間を計算（少し余裕を持たせる）
        turn_time = abs(angle) / self.turning_speed + 0.1  # 0.1秒の余裕
        
        # 回転開始
        self.publisher.publish(twist)
        
        # 回転時間だけ待機（超精度）
        start_time = time.time()
        while time.time() - start_time < turn_time:
            rclpy.spin_once(self, timeout_sec=0.0001)  # 超細かい制御
        
        # 停止
        twist.angular.z = 0.0
        self.publisher.publish(twist)
        
        # 停止を確実にするため少し待機
        time.sleep(0.05)
        
        # 現在の向きを更新
        self.current_theta += angle
        # 角度を-πからπの範囲に正規化
        while self.current_theta > math.pi:
            self.current_theta -= 2 * math.pi
        while self.current_theta < -math.pi:
            self.current_theta += 2 * math.pi
    
    def move_forward(self, distance):
        """指定された距離だけ前進（超精度版）"""
        if distance < 0.001:  # 非常に小さい距離は無視
            return
            
        twist = Twist()
        twist.linear.x = self.drawing_speed
        
        # 移動時間を計算（少し余裕を持たせる）
        move_time = distance / self.drawing_speed + 0.1  # 0.1秒の余裕
        
        # 移動開始
        self.publisher.publish(twist)
        
        # 移動時間だけ待機（超精度）
        start_time = time.time()
        while time.time() - start_time < move_time:
            rclpy.spin_once(self, timeout_sec=0.0001)  # 超細かい制御
        
        # 停止
        twist.linear.x = 0.0
        self.publisher.publish(twist)
        
        # 停止を確実にするため少し待機
        time.sleep(0.05)
    
    def draw_line(self, x1, y1, x2, y2):
        """
        線を描画（シンプル・確実版）
        
        Args:
            x1, y1: 開始点
            x2, y2: 終了点
        """
        self.get_logger().info(f'線を描画: ({x1:.3f}, {y1:.3f}) -> ({x2:.3f}, {y2:.3f})')
        
        # 座標を安全範囲に制限
        x1 = max(1.0, min(10.0, x1))
        y1 = max(1.0, min(10.0, y1))
        x2 = max(1.0, min(10.0, x2))
        y2 = max(1.0, min(10.0, y2))
        
        # ペンを上げて開始点に移動
        self.set_pen(off=True)
        self.simple_move_to(x1, y1)
        
        # ペンを下げて終了点に移動
        self.set_pen(off=False)
        self.simple_move_to(x2, y2)
        
        # 現在位置を更新
        self.current_x = x2
        self.current_y = y2
    
    def simple_move_to(self, target_x, target_y):
        """高精度移動（微調整版）"""
        self.get_logger().info(f'🎯 目標位置: ({target_x:.3f}, {target_y:.3f})')
        
        # 現在の実際の位置を取得
        self.update_current_position()
        self.get_logger().info(f'📍 現在位置: ({self.current_x:.3f}, {self.current_y:.3f}), 角度: {math.degrees(self.current_theta):.1f}度')
        
        # 距離と角度を計算
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        distance = math.sqrt(dx*dx + dy*dy)
        
        if distance < 0.01:
            self.get_logger().info('✅ 目標位置に到達済み')
            return
        
        target_angle = math.atan2(dy, dx)
        self.get_logger().info(f'📐 目標角度: {math.degrees(target_angle):.1f}度, 距離: {distance:.3f}')
        
        # 微調整ループ（最大3回）
        max_attempts = 3
        for attempt in range(max_attempts):
            self.get_logger().info(f'🔄 微調整試行 {attempt + 1}/{max_attempts}')
            
            # 現在の実際の位置を再取得
            self.update_current_position()
            
            # 角度差を計算
            angle_diff = target_angle - self.current_theta
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi
            
            self.get_logger().info(f'📊 角度差: {math.degrees(angle_diff):.1f}度')
            
            # 角度微調整（閾値: 0.05ラジアン ≈ 2.9度）
            if abs(angle_diff) > 0.05:
                self.get_logger().info(f'🔄 角度微調整: {math.degrees(angle_diff):.1f}度')
                self.precise_turn(angle_diff)
                continue
            
            # 距離を再計算
            dx = target_x - self.current_x
            dy = target_y - self.current_y
            distance = math.sqrt(dx*dx + dy*dy)
            
            self.get_logger().info(f'📏 残り距離: {distance:.3f}')
            
            # 距離微調整（閾値: 0.05）
            if distance > 0.05:
                self.get_logger().info(f'🔄 距離微調整: {distance:.3f}')
                self.precise_move_forward(distance)
                continue
            
            # 微調整完了
            self.get_logger().info('✅ 微調整完了')
            break
        
        # 最終位置を更新
        self.update_current_position()
        self.get_logger().info(f'🏁 最終位置: ({self.current_x:.3f}, {self.current_y:.3f}), 角度: {math.degrees(self.current_theta):.1f}度')
    
    def precise_turn(self, angle):
        """高精度回転（詳細ログ付き）"""
        if abs(angle) < 0.001:
            return
        
        # 回転速度を角度に応じて調整
        if abs(angle) < 0.1:  # 小さい角度は低速
            speed = 1.0
        elif abs(angle) < 0.5:  # 中程度の角度は中速
            speed = 1.5
        else:  # 大きい角度は高速
            speed = 2.0
        
        twist = Twist()
        twist.angular.z = speed if angle > 0 else -speed
        
        # 回転時間を計算
        turn_time = abs(angle) / speed
        
        self.get_logger().info(f'🔄 回転開始: {math.degrees(angle):.1f}度, 速度: {speed:.1f}rad/s, 時間: {turn_time:.3f}秒')
        
        # 回転実行
        self.publisher.publish(twist)
        time.sleep(turn_time)
        
        # 停止
        twist.angular.z = 0.0
        self.publisher.publish(twist)
        time.sleep(0.1)
        
        # 角度を更新
        self.current_theta += angle
        while self.current_theta > math.pi:
            self.current_theta -= 2 * math.pi
        while self.current_theta < -math.pi:
            self.current_theta += 2 * math.pi
        
        self.get_logger().info(f'🔄 回転完了: 現在角度 {math.degrees(self.current_theta):.1f}度')
    
    def simple_turn(self, angle):
        """シンプルな回転（後方互換性）"""
        self.precise_turn(angle)
    
    def precise_move_forward(self, distance):
        """高精度前進（詳細ログ付き）"""
        if distance < 0.001:
            return
        
        # 移動速度を距離に応じて調整
        if distance < 0.1:  # 短い距離は低速
            speed = 1.0
        elif distance < 0.5:  # 中程度の距離は中速
            speed = 1.5
        else:  # 長い距離は高速
            speed = 2.0
        
        twist = Twist()
        twist.linear.x = speed
        
        # 移動時間を計算
        move_time = distance / speed
        
        self.get_logger().info(f'🚀 前進開始: 距離 {distance:.3f}, 速度: {speed:.1f}m/s, 時間: {move_time:.3f}秒')
        
        # 移動実行
        self.publisher.publish(twist)
        time.sleep(move_time)
        
        # 停止
        twist.linear.x = 0.0
        self.publisher.publish(twist)
        time.sleep(0.1)
        
        self.get_logger().info(f'🚀 前進完了')
    
    def simple_move_forward(self, distance):
        """シンプルな前進（後方互換性）"""
        self.precise_move_forward(distance)
    
    def draw_lines(self, lines):
        """
        複数の線を描画（高精度版）
        
        Args:
            lines: 線のリスト [[x1, y1, x2, y2], ...]
        """
        self.get_logger().info(f'{len(lines)}本の線を描画開始...')
        
        # 初期位置に移動（ペンを上げて）
        self.set_pen(off=True)
        self.simple_move_to(5.5, 5.5)
        
        for i, line in enumerate(lines):
            if len(line) >= 4:
                x1, y1, x2, y2 = line[:4]
                
                # 色を変更（5本ごと）
                if i % 5 == 0:
                    color = self.colors[self.current_color_index % len(self.colors)]
                    self.set_pen(r=color[0], g=color[1], b=color[2], width=2, off=False)
                    self.current_color_index += 1
                
                self.draw_line(x1, y1, x2, y2)
                
                # 進捗を表示
                if (i + 1) % 10 == 0:
                    self.get_logger().info(f'進捗: {i + 1}/{len(lines)} 線完了')
        
        self.get_logger().info('✅ すべての線の描画が完了しました')
    
    def clear_screen(self):
        """画面をクリア"""
        self.get_logger().info('画面をクリア中...')
        
        # ペンを上げる
        self.set_pen(off=True)
        
        # 画面の端に移動してクリア
        self.move_to_position(0, 0, draw=False)
        self.move_to_position(11, 11, draw=False)
        self.move_to_position(5.5, 5.5, draw=False)
        
        self.get_logger().info('画面クリア完了')
    
    def draw_sample_pattern(self):
        """サンプルパターンを描画"""
        self.get_logger().info('サンプルパターンを描画中...')
        
        # ペンを上げて開始位置に移動
        self.set_pen(off=True)
        self.move_to_position(2, 2, draw=False)
        
        # 四角形を描画
        self.set_pen(off=False)
        self.move_to_position(8, 2, draw=True)
        self.move_to_position(8, 8, draw=True)
        self.move_to_position(2, 8, draw=True)
        self.move_to_position(2, 2, draw=True)
        
        # 対角線を描画
        self.move_to_position(8, 8, draw=True)
        self.move_to_position(2, 2, draw=True)
        self.move_to_position(8, 2, draw=True)
        self.move_to_position(2, 8, draw=True)
        
        self.get_logger().info('サンプルパターン描画完了')

def main():
    rclpy.init()
    
    drawer = TurtleDrawer()
    
    try:
        # サンプルパターンを描画
        drawer.draw_sample_pattern()
        
        # 少し待機
        time.sleep(2)
        
        # 画面をクリア
        drawer.clear_screen()
        
    except Exception as e:
        drawer.get_logger().error(f'エラー: {e}')
    finally:
        drawer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
