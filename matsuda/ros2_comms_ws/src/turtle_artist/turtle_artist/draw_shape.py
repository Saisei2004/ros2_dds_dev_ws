import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import math
from turtlesim.srv import SetPen, TeleportAbsolute
from std_srvs.srv import Empty  # Clearの代わりにこれをインポート
from ament_index_python.packages import get_package_share_directory
import os
import asyncio

# --- Constants ---
TURTLE_MAX_X = 11.0
TURTLE_MAX_Y = 11.0
PEN_COLOR = [255, 255, 255] # White
PEN_WIDTH = 2

class PixelPainter(Node):
    """
    Draws image contours in Turtlesim using absolute coordinates and angles.
    """
    def __init__(self, image_path):
        # New Node Name
        super().__init__('pixel_painter_node')
        self.get_logger().info(f"Loading image: {image_path}")
        self.image_path = image_path

        self.set_pen_client = self.create_client(SetPen, '/turtle1/set_pen')
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        # サービス・クライアントを作成する際の型を'Empty'に変更
        self.clear_client = self.create_client(Empty, '/clear')

    async def wait_for_services(self):
        """Waits until all required services are available."""
        for client, name in [(self.set_pen_client, 'set_pen'), (self.teleport_client, 'teleport_absolute'), (self.clear_client, 'clear')]:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'{name} service not available, waiting...')
        self.get_logger().info('All services are ready.')

    async def call_set_pen(self, r, g, b, width, off):
        req = SetPen.Request()
        req.r, req.g, req.b, req.width, req.off = int(r), int(g), int(b), int(width), int(off)
        await self.set_pen_client.call_async(req)

    async def call_teleport(self, x, y, theta):
        req = TeleportAbsolute.Request()
        req.x, req.y, req.theta = float(x), float(y), float(theta)
        await self.teleport_client.call_async(req)
    
    async def call_clear(self):
        # 呼び出すリクエストを'Empty.Request()'に変更
        await self.clear_client.call_async(Empty.Request())

    def get_contours_from_image(self):
        img = cv2.imread(self.image_path)
        if img is None:
            self.get_logger().error(f"Image file not found: {self.image_path}")
            return None, 0, 0
        
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(img_gray, 50, 150)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return sorted(contours, key=cv2.contourArea, reverse=True), img.shape[0], img.shape[1]

    def transform_coords(self, point, img_h, img_w):
        turtle_x = (point[0] / img_w) * TURTLE_MAX_X
        turtle_y = ((img_h - point[1]) / img_h) * TURTLE_MAX_Y
        return turtle_x, turtle_y
        
    async def draw(self):
        """The main drawing execution logic."""
        await self.wait_for_services()
        self.get_logger().info("Starting draw sequence...")

        await self.call_clear()
        await self.call_set_pen(0, 0, 0, 0, 1)

        contours, img_h, img_w = self.get_contours_from_image()
        if contours is None:
            rclpy.shutdown()
            return

        for contour in contours:
            points = [p[0] for p in contour]
            
            for i, point in enumerate(points):
                next_point = points[(i + 1) % len(points)]
                
                current_turtle_pt = self.transform_coords(point, img_h, img_w)
                next_turtle_pt = self.transform_coords(next_point, img_h, img_w)

                dx = next_turtle_pt[0] - current_turtle_pt[0]
                dy = next_turtle_pt[1] - current_turtle_pt[1]
                theta = math.atan2(dy, dx)

                if i == 0:
                    await self.call_teleport(current_turtle_pt[0], current_turtle_pt[1], theta)
                    await self.call_set_pen(PEN_COLOR[0], PEN_COLOR[1], PEN_COLOR[2], PEN_WIDTH, 0)
                
                await self.call_teleport(next_turtle_pt[0], next_turtle_pt[1], theta)

            await self.call_set_pen(0, 0, 0, 0, 1)

        self.get_logger().info("Drawing complete!")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    # Path discovery now uses the new package name 'turtle_artist'
    package_share_directory = get_package_share_directory('turtle_artist')
    image_path = os.path.join(package_share_directory, 'resource', 'shape.png')
    
    node = PixelPainter(image_path)
    
    try:
        asyncio.run(node.draw())
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()