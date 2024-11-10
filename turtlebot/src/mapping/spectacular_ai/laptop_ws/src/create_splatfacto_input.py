import json
import os
import glob
import shutil
import argparse
import numpy as np
from scipy.spatial.transform import Rotation as R

def load_camera_info(camera_info_path):
    """Load camera intrinsics from camera_info.json."""
    with open(camera_info_path, 'r') as f:
        camera_info = json.load(f)
    return camera_info["fl_x"], camera_info["fl_y"], camera_info["cx"], camera_info["cy"], camera_info["height"], camera_info["width"]

def create_splatfacto_structure(output_dir):
    """Create the folder structure required by SplatFacto in Nerfstudio."""
    images_dir = os.path.join(output_dir, "images")
    os.makedirs(images_dir, exist_ok=True)
    return images_dir

def copy_images(image_dir, images_output_dir):
    """Copy keyframe images to the SplatFacto 'images' directory."""
    for image_path in sorted(glob.glob(os.path.join(image_dir, "keyframe_*.png"))):
        shutil.copy(image_path, images_output_dir)

def pose_to_transform_matrix(position, orientation):
    """Convert position and orientation to a 4x4 transformation matrix."""
    translation = np.array([position["x"], position["y"], position["z"]])
    rotation = R.from_quat([orientation["x"], orientation["y"], orientation["z"], orientation["w"]])
    transform_matrix = np.eye(4)
    transform_matrix[:3, :3] = rotation.as_matrix()
    transform_matrix[:3, 3] = translation
    return transform_matrix.tolist()

def create_transforms_json(image_dir, images_output_dir, camera_info_path, output_dir):
    """Generate transforms.json for NeRFStudio using existing pose files and camera_info.json."""
    fl_x, fl_y, cx, cy, height, width = load_camera_info(camera_info_path)
    frames = []

    for image_path in sorted(glob.glob(os.path.join(image_dir, "keyframe_*.png"))):
        frame_id = os.path.splitext(os.path.basename(image_path))[0]
        pose_file = os.path.join(image_dir, f"{frame_id}.json")

        if os.path.exists(pose_file):
            with open(pose_file, 'r') as f:
                pose_data = json.load(f)

            # Convert position and orientation to transform matrix
            transform_matrix = pose_to_transform_matrix(pose_data["position"], pose_data["orientation"])
            frames.append({
                "file_path": os.path.relpath(os.path.join(images_output_dir, os.path.basename(image_path)), output_dir),
                "transform_matrix": transform_matrix,
                "intrinsics": {
                    "fl_x": fl_x,
                    "fl_y": fl_y,
                    "cx": cx,
                    "cy": cy,
                    "h": height,
                    "w": width
                }
            })

    transforms_data = {
        "fl_x": fl_x,
        "fl_y": fl_y,
        "cx": cx,
        "cy": cy,
        "h": height,
        "w": width,
        "frames": frames
    }

    transforms_path = os.path.join(output_dir, "transforms.json")
    with open(transforms_path, 'w') as f:
        json.dump(transforms_data, f, indent=4)
    
    print(f"transforms.json created at {transforms_path}")

def copy_ply_file(ply_file_path, output_dir):
    """Copy the .ply file to the output directory."""
    if os.path.exists(ply_file_path):
        shutil.copy(ply_file_path, output_dir)
        print(f"PLY file copied to {output_dir}")
    else:
        print(f"PLY file not found at {ply_file_path}")

def main():
    parser = argparse.ArgumentParser(description="Prepare dataset for Nerfstudio SplatFacto.")
    parser.add_argument("--image-dir", required=True, help="Directory containing the keyframe images and pose files.")
    parser.add_argument("--output-dir", required=True, help="Root directory for the SplatFacto dataset structure.")
    parser.add_argument("--camera-info", required=True, help="Path to camera_info.json containing camera intrinsics.")
    parser.add_argument("--ply-file", required=True, help="Path to the .ply file to be copied.")
    
    args = parser.parse_args()

    # Create required folder structure for Nerfstudio's SplatFacto model
    images_output_dir = create_splatfacto_structure(args.output_dir)

    # Copy keyframe images into the appropriate location
    copy_images(args.image_dir, images_output_dir)

    # Generate transforms.json and place it in the output directory
    create_transforms_json(args.image_dir, images_output_dir, args.camera_info, args.output_dir)

    # Copy the .ply file to the output directory
    copy_ply_file(args.ply_file, args.output_dir)
    
    print("Dataset preparation for SplatFacto completed.")

if __name__ == "__main__":
    main()
