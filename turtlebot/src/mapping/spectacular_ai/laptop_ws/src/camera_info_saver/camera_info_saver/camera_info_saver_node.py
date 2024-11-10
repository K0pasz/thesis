import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
import json

class CameraInfoSaver(Node):
    def __init__(self, save_path="camera_info.json"):
        super().__init__('camera_info_saver')
        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            '/slam/camera_info',
            self.camera_info_callback,
            10
        )
        self.save_path = save_path

    def camera_info_callback(self, msg):
        # Extract the focal lengths and principal points from the intrinsics matrix
        camera_data = {
            "height": msg.height,
            "width": msg.width,
            "fl_x": msg.k[0],  # Focal length x
            "fl_y": msg.k[4],  # Focal length y
            "cx": msg.k[2],    # Principal point x
            "cy": msg.k[5],    # Principal point y
            "distortion_model": msg.distortion_model,
            "intrinsics": list(msg.k[:9])
        }
        
        with open(self.save_path, 'w') as f:
            json.dump(camera_data, f, indent=4)
        
        self.get_logger().info(f"Camera info saved to {self.save_path}")
        rclpy.shutdown()  # Shutdown after saving once

def main(args=None):
    rclpy.init(args=args)
    camera_info_saver = CameraInfoSaver()
    rclpy.spin(camera_info_saver)

if __name__ == '__main__':
    main()
