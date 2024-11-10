import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
import struct
import os

class PointCloudSaver(Node):
    def __init__(self):
        super().__init__('pointcloud_saver')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/slam/pointcloud',
            self.pointcloud_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.ply_file_path = 'sparse_pc.ply'
        self.header_written = False

    def pointcloud_callback(self, msg):
        points = self.pointcloud2_to_array(msg)
        self.save_to_ply(points)
        self.get_logger().info(f"Point cloud saved to {self.ply_file_path}")

    def pointcloud2_to_array(self, cloud_msg):
        points = []
        for i in range(cloud_msg.width * cloud_msg.height):
            point_offset = i * cloud_msg.point_step
            x, y, z = struct.unpack_from('fff', cloud_msg.data, point_offset)
            points.append((x, y, z))
        return np.array(points)

    def save_to_ply(self, points):
        # Check if header needs to be written
        if not self.header_written:
            with open(self.ply_file_path, 'w') as f:
                f.write("ply\n")
                f.write("format ascii 1.0\n")
                f.write(f"element vertex {len(points)}\n")
                f.write("property float x\n")
                f.write("property float y\n")
                f.write("property float z\n")
                f.write("end_header\n")
            self.header_written = True
        else:
            with open(self.ply_file_path, 'a') as f:
                for point in points:
                    f.write(f"{point[0]} {point[1]} {point[2]}\n")

def main(args=None):
    rclpy.init(args=args)
    pointcloud_saver = PointCloudSaver()
    rclpy.spin(pointcloud_saver)
    pointcloud_saver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
