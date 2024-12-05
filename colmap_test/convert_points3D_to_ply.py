import numpy as np
import struct
import plyfile
import os

def read_points3D_binary(file_path):
    """
    Read the points3D.bin file from COLMAP and return the points, colors, and visibility data.
    """
    points3D = {}
    with open(file_path, "rb") as f:
        num_points = struct.unpack("<Q", f.read(8))[0]
        for _ in range(num_points):
            point_id = struct.unpack("<Q", f.read(8))[0]
            xyz = struct.unpack("<ddd", f.read(24))
            rgb = struct.unpack("<BBB", f.read(3))
            error = struct.unpack("<d", f.read(8))[0]
            track_length = struct.unpack("<Q", f.read(8))[0]
            track_data = f.read(8 * track_length)  # Skip track data
            points3D[point_id] = {
                "xyz": np.array(xyz),
                "rgb": np.array(rgb, dtype=np.uint8),
                "error": error,
                "track_length": track_length,
            }
    return points3D

def transform_point_cloud(points3D, transformation_matrix):
    """
    Apply a 4x4 transformation matrix to the point cloud.
    """
    transformed_points3D = {}
    for point_id, data in points3D.items():
        xyz = np.append(data["xyz"], 1.0)  # Convert to homogeneous coordinates
        transformed_xyz = transformation_matrix @ xyz
        transformed_points3D[point_id] = {
            "xyz": transformed_xyz[:3],  # Back to 3D
            "rgb": data["rgb"],
        }
    return transformed_points3D

def write_ply(points3D, output_file):
    """
    Write a .ply file from points3D data.
    """
    vertices = []
    for point_id, data in points3D.items():
        xyz = data["xyz"]
        rgb = data["rgb"]
        vertices.append((*xyz, *rgb))

    vertex_array = np.array(vertices, dtype=[
        ("x", "f4"), ("y", "f4"), ("z", "f4"),
        ("red", "u1"), ("green", "u1"), ("blue", "u1")
    ])

    ply = plyfile.PlyData(
        [plyfile.PlyElement.describe(vertex_array, "vertex")],
        text=True
    )
    ply.write(output_file)
    print(f"PLY file written to {output_file}")

# Define the rotations
R_x = np.array([
    [1, 0, 0],
    [0, 0, -1],
    [0, 1, 0]
])

R_z = np.array([
    [0, 1, 0],
    [-1, 0, 0],
    [0, 0, 1]
])

R_z2 = np.array([
    [-1.0000000, -0.0000000,  0.0000000],
    [0.0000000, -1.0000000,  0.0000000],
    [0.0000000,  0.0000000,  1.0000000]
])

# Combine the rotations
T = R_z @ R_z2 @ R_x

# Convert to homogeneous 4x4 matrix
T_homogeneous = np.eye(4)
T_homogeneous[:3, :3] = T

# Specify paths
input_bin = "/home/laci/EXPERIMENTS/colmap_generation/sparse/model/triangulated/points3D.bin"
output_ply = "/home/laci/EXPERIMENTS/colmap_generation/sparse/model/triangulated/points3D_transformed.ply"


# Convert
if os.path.exists(input_bin):
    points3D = read_points3D_binary(input_bin)
    transformed_points3D = transform_point_cloud(points3D, T_homogeneous)
    write_ply(transformed_points3D, output_ply)
else:
    print(f"Input file {input_bin} does not exist.")
