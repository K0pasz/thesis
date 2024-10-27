import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import os

class KeyframeImageSaver(Node):
    def __init__(self, save_directory="keyframes"):
        super().__init__('keyframe_image_saver')
        self.image_subscription = self.create_subscription(
            Image,
            '/slam/left',
            self.image_callback,
            10)
        self.keyframe_subscription = self.create_subscription(
            PoseStamped,
            '/slam/keyframe',
            self.keyframe_callback,
            10)
        self.bridge = CvBridge()
        self.save_directory = save_directory
        os.makedirs(self.save_directory, exist_ok=True)
        self.latest_image = None
        self.image_count = 0

    def image_callback(self, msg):
        # Save the latest image for potential saving
        self.latest_image = msg

    def keyframe_callback(self, msg):
        if self.latest_image:
            try:
                # Convert ROS Image message to OpenCV format
                cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, "bgr8")
                
                # Save image with the pose timestamp
                filename = os.path.join(self.save_directory, f"keyframe_{self.image_count:04d}.png")
                cv2.imwrite(filename, cv_image)
                self.image_count += 1
                self.get_logger().info(f"Saved image at keyframe: {filename}")
            except Exception as e:
                self.get_logger().error(f"Failed to save image: {e}")

def main(args=None):
    rclpy.init(args=args)
    keyframe_image_saver = KeyframeImageSaver()
    rclpy.spin(keyframe_image_saver)
    keyframe_image_saver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
