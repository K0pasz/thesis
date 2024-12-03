import json
import os
from glob import glob

def convert_pose_to_colmap(position, orientation):
    """
    Converts ROS pose to COLMAP coordinate system.
    """
    # Convert position (negate Y and Z)
    tx, ty, tz = position["x"], -position["y"], -position["z"]
    
    # Convert quaternion (negate Y and Z of orientation)
    qw, qx, qy, qz = orientation["w"], orientation["x"], -orientation["y"], -orientation["z"]

    return (qw, qx, qy, qz, tx, ty, tz)

def create_images_txt(image_dir, pose_dir, output_path):
    """
    Create images.txt for COLMAP using image files and corresponding pose JSONs.
    """
    image_files = sorted(glob(os.path.join(image_dir, "*.png")))
    pose_files = sorted(glob(os.path.join(pose_dir, "*.json")))

    with open(output_path, "w") as f:
        for idx, (image_file, pose_file) in enumerate(zip(image_files, pose_files)):
            with open(pose_file, "r") as pose_f:
                pose_data = json.load(pose_f)

            position = pose_data["position"]
            orientation = pose_data["orientation"]

            qw, qx, qy, qz, tx, ty, tz = convert_pose_to_colmap(position, orientation)
            image_name = os.path.basename(image_file)

            # Write metadata line for the image
            f.write(f"{idx + 1} {qw} {qx} {qy} {qz} {tx} {ty} {tz} 1 {image_name}\n")
            
            # Write an empty line as required by COLMAP
            f.write("\n")

    print(f"images.txt created at {output_path}")

# Usage
create_images_txt("/home/laci/mapping1_copy/keyframes", "/home/laci/mapping1_copy/keyframes", "/home/laci/EXPERIMENTS/colmap_generation/sparse/model/images.txt")
