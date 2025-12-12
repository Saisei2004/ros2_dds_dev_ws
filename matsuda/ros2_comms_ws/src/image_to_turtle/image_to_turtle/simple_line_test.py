#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen
import time
import math

class SimpleLineTest(Node):
    def __init__(self):
        super().__init__('simple_line_test')
        
        # パブリッシャーとクライアントを作成
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.set_pen_client = self.create_client(SetPen, '/turtle1/set_pen')
        
        # 現在の位置と向き
        self.current_x = 5.5
        self.current_y = 5.5
        self.current_theta = 0.0
        
        self.get_logger().info('シンプル線描画テストが初期化されました')
        
        # サービスが利用可能になるまで待機
        self.wait_for_services()
    
    def wait_for_services(self):
        """必要なサービスが利用可能になるまで待機"""
        self.get_logger().info('サービスを待機中...')
        
        while not self.set_pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('set_penサービスを待機中...')
        
        self.get_logger().info('サービスが利用可能になりました')
    
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
            self.get_logger().info(f'ペン設定: {"OFF" if off else "ON"}')
        else:
            self.get_logger().error('ペン設定に失敗しました')
    
    def move_to(self, target_x, target_y, draw=True):
        """指定された位置に移動"""
        self.get_logger().info(f'移動: ({self.current_x:.2f}, {self.current_y:.2f}) -> ({target_x:.2f}, {target_y:.2f})')
        
        # ペンの状態を設定
        if draw:
            self.set_pen(off=False)
        else:
            self.set_pen(off=True)
        
        # 距離と角度を計算
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        distance = math.sqrt(dx*dx + dy*dy)
        
        if distance < 0.01:
            return
        
        target_angle = math.atan2(dy, dx)
        
        # 回転
        angle_diff = target_angle - self.current_theta
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        if abs(angle_diff) > 0.1:
            self.turn(angle_diff)
        
        # 直進
        self.move_forward(distance)
        
        # 位置を更新
        self.current_x = target_x
        self.current_y = target_y
        self.current_theta = target_angle
    
    def turn(self, angle):
        """指定された角度だけ回転"""
        if abs(angle) < 0.01:
            return
        
        twist = Twist()
        twist.angular.z = 2.0 if angle > 0 else -2.0
        
        # 回転時間を計算
        turn_time = abs(angle) / 2.0
        
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
    
    def move_forward(self, distance):
        """指定された距離だけ前進"""
        if distance < 0.01:
            return
        
        twist = Twist()
        twist.linear.x = 2.0
        
        # 移動時間を計算
        move_time = distance / 2.0
        
        # 移動実行
        self.publisher.publish(twist)
        time.sleep(move_time)
        
        # 停止
        twist.linear.x = 0.0
        self.publisher.publish(twist)
        time.sleep(0.1)
    
    def draw_square(self):
        """四角形を描画"""
        self.get_logger().info('四角形を描画開始')
        
        # 開始位置に移動（ペンを上げて）
        self.move_to(3.0, 3.0, draw=False)
        
        # 四角形を描画
        self.move_to(7.0, 3.0, draw=True)  # 下辺
        self.move_to(7.0, 7.0, draw=True)  # 右辺
        self.move_to(3.0, 7.0, draw=True)  # 上辺
        self.move_to(3.0, 3.0, draw=True)  # 左辺
        
        self.get_logger().info('四角形描画完了')
    
    def draw_triangle(self):
        """三角形を描画"""
        self.get_logger().info('三角形を描画開始')
        
        # 開始位置に移動（ペンを上げて）
        self.move_to(5.0, 2.0, draw=False)
        
        # 三角形を描画
        self.move_to(8.0, 2.0, draw=True)  # 底辺
        self.move_to(6.5, 5.0, draw=True)  # 右辺
        self.move_to(5.0, 2.0, draw=True)  # 左辺
        
        self.get_logger().info('三角形描画完了')
    
    def run_test(self):
        """テストを実行"""
        self.get_logger().info('=== シンプル線描画テスト開始 ===')
        
        try:
            # 四角形を描画
            self.draw_square()
            time.sleep(2)
            
            # 三角形を描画
            self.draw_triangle()
            time.sleep(2)
            
            self.get_logger().info('=== テスト完了 ===')
            
        except Exception as e:
            self.get_logger().error(f'テスト実行中にエラー: {e}')

def main(args=None):
    rclpy.init(args=args)
    
    test = SimpleLineTest()
    
    try:
        test.run_test()
    except Exception as e:
        test.get_logger().error(f'エラー: {e}')
    finally:
        test.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
