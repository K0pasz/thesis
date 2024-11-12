import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
import numpy as np
import struct
import os

class PointCloudSaver(Node):
    def __init__(self, ply_file_path="sparse_pc.ply"):
        super().__init__('pointcloud_saver')
        self.pointcloud_subscription = self.create_subscription(
            PointCloud2,
            '/slam/pointcloud',
            self.pointcloud_callback,
            10
        )
        self.keyframe_subscription = self.create_subscription(
            PoseStamped,
            '/slam/keyframe',
            self.keyframe_callback,
            10
        )
        self.ply_file_path = ply_file_path
        self.header_written = False
        self.latest_pointcloud = None  # Store the latest point cloud message

    def pointcloud_callback(self, msg):
        # Store the latest point cloud message for potential saving
        self.latest_pointcloud = msg

    def keyframe_callback(self, msg):
        # When a keyframe message is received, save the latest point cloud
        if self.latest_pointcloud:
            points = self.pointcloud2_to_array(self.latest_pointcloud)
            self.append_to_ply(points)
            self.get_logger().info(f"Point cloud data appended to {self.ply_file_path}")

    def pointcloud2_to_array(self, cloud_msg):
        points = []
        for i in range(cloud_msg.width * cloud_msg.height):
            point_offset = i * cloud_msg.point_step
            x, y, z = struct.unpack_from('fff', cloud_msg.data, point_offset)
            points.append((x, y, z))
        return np.array(points)

    def append_to_ply(self, points):
        # Write header only if it hasn't been written yet
        if not self.header_written:
            with open(self.ply_file_path, 'w') as f:
                f.write("ply\n")
                f.write("format ascii 1.0\n")
                f.write("element vertex 0\n")  # Temporary; will update with vertex count
                f.write("property float x\n")
                f.write("property float y\n")
                f.write("property float z\n")
                f.write("end_header\n")
            self.header_written = True

        # Append points and update vertex count in the header
        with open(self.ply_file_path, 'a') as f:
            for point in points:
                f.write(f"{point[0]} {point[1]} {point[2]}\n")

        # Update vertex count in the header
        self.update_vertex_count(len(points))

    def update_vertex_count(self, new_points_count):
        with open(self.ply_file_path, 'r+') as f:
            lines = f.readlines()
            for i, line in enumerate(lines):
                if line.startswith("element vertex"):
                    # Update the vertex count
                    current_count = int(line.split()[2])
                    updated_count = current_count + new_points_count
                    lines[i] = f"element vertex {updated_count}\n"
                    break
            f.seek(0)
            f.writelines(lines)

def main(args=None):
    rclpy.init(args=args)
    pointcloud_saver = PointCloudSaver()
    rclpy.spin(pointcloud_saver)
    pointcloud_saver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
