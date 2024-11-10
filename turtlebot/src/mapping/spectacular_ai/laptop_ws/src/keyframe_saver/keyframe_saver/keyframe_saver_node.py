import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import os
import json

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
        self.latest_pose = None
        self.image_count = 0

    def image_callback(self, msg):
        # Save the latest image for potential saving
        self.latest_image = msg

    def keyframe_callback(self, msg):
        self.latest_pose = msg.pose  # Store the latest pose
        
        if self.latest_image and self.latest_pose:
            try:
                # Convert ROS Image message to OpenCV format
                cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, "bgr8")

                # Save image with the pose timestamp
                image_filename = os.path.join(self.save_directory, f"keyframe_{self.image_count:04d}.png")
                cv2.imwrite(image_filename, cv_image)

                # Save pose to JSON
                pose_filename = os.path.join(self.save_directory, f"keyframe_{self.image_count:04d}.json")
                pose_data = {
                    "position": {
                        "x": self.latest_pose.position.x,
                        "y": self.latest_pose.position.y,
                        "z": self.latest_pose.position.z
                    },
                    "orientation": {
                        "x": self.latest_pose.orientation.x,
                        "y": self.latest_pose.orientation.y,
                        "z": self.latest_pose.orientation.z,
                        "w": self.latest_pose.orientation.w
                    }
                }
                with open(pose_filename, 'w') as pose_file:
                    json.dump(pose_data, pose_file, indent=4)

                self.image_count += 1
                self.get_logger().info(f"Saved image and pose at keyframe: {image_filename}, {pose_filename}")
            except Exception as e:
                self.get_logger().error(f"Failed to save image or pose: {e}")

def main(args=None):
    rclpy.init(args=args)
    keyframe_image_saver = KeyframeImageSaver()
    rclpy.spin(keyframe_image_saver)
    keyframe_image_saver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
