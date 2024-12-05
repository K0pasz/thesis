import numpy as np
from scipy.spatial.transform import Rotation as R

def convert_pose_to_colmap(position, orientation):
    """
    Converts a robotics pose to COLMAP coordinate system.
    """
    translation = np.array([position["x"], position["y"], position["z"]])
    rotation = R.from_quat([orientation["x"], orientation["y"], orientation["z"], orientation["w"]])

    # Create pose_c2m (camera-to-world transform)
    pose_c2m = np.eye(4)
    pose_c2m[:3, :3] = rotation.as_matrix()
    pose_c2m[:3, 3] = translation

    # Invert to get pose_m2c (world-to-camera transform)
    pose_m2c = np.linalg.inv(pose_c2m)

    # Extract new rotation and translation
    new_R = pose_m2c[:3, :3]
    new_t = pose_m2c[:3, 3]

    # Convert rotation matrix to quaternion
    new_qvec = R.from_matrix(new_R).as_quat()  # Output as (qx, qy, qz, qw)

    # Rearrange quaternion to COLMAP format: (qw, qx, qy, qz)
    new_qvec = [new_qvec[3], new_qvec[0], new_qvec[1], new_qvec[2]]

    return new_qvec, new_t

def create_images_txt(image_dir, pose_dir, output_path):
    """
    Create images.txt for COLMAP using image files and corresponding pose JSONs.
    """
    from glob import glob
    import os
    import json

    image_files = sorted(glob(os.path.join(image_dir, "*.png")))
    pose_files = sorted(glob(os.path.join(pose_dir, "*.json")))

    with open(output_path, "w") as f:
        for idx, (image_file, pose_file) in enumerate(zip(image_files, pose_files)):
            with open(pose_file, "r") as pose_f:
                pose_data = json.load(pose_f)

            position = pose_data["position"]
            orientation = pose_data["orientation"]

            # Convert pose
            new_qvec, new_t = convert_pose_to_colmap(position, orientation)

            # Write to images.txt
            image_name = os.path.basename(image_file)
            f.write(f"{idx + 1} {new_qvec[0]} {new_qvec[1]} {new_qvec[2]} {new_qvec[3]} {new_t[0]} {new_t[1]} {new_t[2]} 1 {image_name}\n")
            f.write("\n")  # Empty line as required by COLMAP

    print(f"images.txt created at {output_path}")

# Usage
create_images_txt(
    "/home/laci/mapping1_copy/keyframes", 
    "/home/laci/mapping1_copy/keyframes", 
    "/home/laci/EXPERIMENTS/colmap_generation/sparse/model/images.txt"
)
