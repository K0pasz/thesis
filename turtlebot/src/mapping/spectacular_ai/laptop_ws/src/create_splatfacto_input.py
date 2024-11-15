import json
import os
import glob
import shutil
import argparse
import numpy as np
from scipy.spatial.transform import Rotation as R

# Vision to graphics transformation matrix
T_vision_to_graphics = np.diag([1, -1, -1, 1])

# Robotics to vision transformation matrix
T_vision_to_robotics = np.array([
    [ 0.0,  0.0,  1.0, 0.0],
    [-1.0,  0.0,  0.0, 0.0],
    [ 0.0, -1.0,  0.0, 0.0],
    [ 0.0,  0.0,  0.0, 1.0]
])

T_robotics_to_vision = np.linalg.inv(T_vision_to_robotics)


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
    
    # Initial transformation matrix
    pose_robotics = np.eye(4)
    pose_robotics[:3, :3] = rotation.as_matrix()
    pose_robotics[:3, 3] = translation
    
    # Apply the transformation
    #transform_matrix = T_robotics_to_vision @ transform_matrix # camera2world (pose)


    #transform_matrix = transform_matrix @ T_vision_to_robotics

    
    #transform_matrix = T_vision_to_graphics @ transform_matrix

    
    #transform_matrix = transform_matrix @ T_vision_to_robotics
    #transform_matrix = np.linalg.inv(transform_matrix) # world2camera

    #transform_matrix = T_robotics_to_vision @ transform_matrix
    #transform_matrix = transform_matrix @ T_vision_to_graphics


    pose_vision = T_vision_to_robotics @ pose_robotics
    pose_graphics = pose_vision @ T_vision_to_graphics
    
    
    #pose_graphics = pose_new @ T_vision_to_graphics
    #pose_vision = np.eye(4)
    #pose_vision = pose_robotics @ T_robotics_to_vision ???

    return pose_graphics.tolist()

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
        "frames": frames,
        "ply_file_path": "sparse_pc.ply"
    }

    transforms_path = os.path.join(output_dir, "transforms.json")
    with open(transforms_path, 'w') as f:
        json.dump(transforms_data, f, indent=4)
    
    print(f"transforms.json created at {transforms_path}")


def copy_ply_file(ply_file_path, output_dir):
    """Copy and transform the .ply file points to the output directory."""
    if not os.path.exists(ply_file_path):
        print(f"PLY file not found at {ply_file_path}")
        return
    
    transformed_ply_path = os.path.join(output_dir, "sparse_pc.ply")
    
    # Robotics to vision transformation matrix

    with open(ply_file_path, 'r') as f:
        lines = f.readlines()

    # Find the starting point of vertex data
    header_lines = []
    vertex_data = []
    header_complete = False

    for line in lines:
        if line.startswith("end_header"):
            header_complete = True
            header_lines.append(line)
            continue
        if header_complete:
            # Skip empty lines or any lines that do not contain three values (for vertices)
            parts = line.strip().split()
            if len(parts) == 3:  # Ensure there are exactly three values (x, y, z)
                vertex_data.append(parts)
        else:
            header_lines.append(line)

    # Total number of vertices to process
    total_vertices = len(vertex_data)

    # Transform each vertex point and save to a new file
    with open(transformed_ply_path, 'w') as f:
        f.writelines(header_lines)
        for idx, line in enumerate(vertex_data):
            x, y, z = map(float, line)
            vertex_coords = np.array([x, y, z, 1])
            
            # EREDETI
            #transformed_point = T_robotics_to_vision @ vertex_coords
            #transformed_point = transformed_point @ T_vision_to_graphics

            #transformed_point = vertex_coords @ T_vision_to_graphics

            #transformed_point = T_vision_to_graphics @ vertex_coords

            #transformed_point = T_robotics_to_vision @ vertex_coords

            #transformed_point = vertex_coords @ T_robotics_to_vision


            #transformed_point = T_robotics_to_vision @ vertex_coords
            #transformed_point = transformed_point @ T_vision_to_graphics

            #transformed_point = vertex_coords @ T_robotics_to_vision
            #transformed_point = transformed_point @ T_vision_to_graphics

            #transformed_point = T_robotics_to_vision @ vertex_coords
            #transformed_point = T_vision_to_graphics @ transformed_point

            #transformed_point = vertex_coords @ T_robotics_to_vision
            #transformed_point = T_vision_to_graphics @ transformed_point

            #T_full = T_vision_to_graphics @ T_robotics_to_vision

            #T_full = np.linalg.inv(T_vision_to_graphics) @ T_robotics_to_vision

            #transformed_point = T_full @ vertex_coords

            transformed_point = T_vision_to_robotics @ vertex_coords

            f.write(f"{transformed_point[0]} {transformed_point[1]} {transformed_point[2]}\n")
            
            # Print progress update every 1% of total vertices processed
            if (idx + 1) % (total_vertices // 100) == 0:
                progress = (idx + 1) / total_vertices * 100
                print(f"Processing vertices: {progress:.2f}% ({idx + 1}/{total_vertices})")

    print(f"Transformed PLY file saved to {transformed_ply_path}")
    

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
