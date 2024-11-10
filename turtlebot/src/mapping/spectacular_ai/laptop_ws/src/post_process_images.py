import cv2
import os
import glob
import argparse
import numpy as np
import json

def is_blurry(image_path, threshold):
    """Check if an image is blurry using the variance of the Laplacian."""
    image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if image is None:
        return False
    laplacian_var = cv2.Laplacian(image, cv2.CV_64F).var()
    return laplacian_var < threshold

def is_overexposed(image_path, brightness_threshold):
    """Check if an image is overexposed based on brightness levels."""
    image = cv2.imread(image_path)
    if image is None:
        return False
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    mean_brightness = np.mean(gray)
    return mean_brightness > brightness_threshold

def delete_blurry_and_overexposed_images(directory, blur_threshold, brightness_threshold):
    """Delete images and corresponding JSON files if they are blurry or overexposed."""
    images = sorted(glob.glob(os.path.join(directory, "*.png")))
    for image_path in images:
        pose_path = image_path.replace(".png", ".json")
        if is_blurry(image_path, blur_threshold) or is_overexposed(image_path, brightness_threshold):
            print(f"Deleting blurry or overexposed image and pose: {image_path} and {pose_path}")
            os.remove(image_path)
            if os.path.exists(pose_path):
                os.remove(pose_path)

def renumber_images_and_poses(directory):
    """Renumber images and pose JSON files sequentially after deletion."""
    images = sorted(glob.glob(os.path.join(directory, "*.png")))
    for i, image_path in enumerate(images):
        new_image_name = os.path.join(directory, f"keyframe_{i:04d}.png")
        new_pose_name = os.path.join(directory, f"keyframe_{i:04d}.json")
        
        if image_path != new_image_name:
            os.rename(image_path, new_image_name)
            print(f"Renaming {image_path} to {new_image_name}")

        # Rename corresponding JSON pose file
        old_pose_path = image_path.replace(".png", ".json")
        if os.path.exists(old_pose_path):
            os.rename(old_pose_path, new_pose_name)
            print(f"Renaming {old_pose_path} to {new_pose_name}")

def main(directory, blur_threshold, brightness_threshold):
    """Run the post-processing on the specified directory."""
    delete_blurry_and_overexposed_images(directory, blur_threshold, brightness_threshold)
    renumber_images_and_poses(directory)
    print("Post-processing completed.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Post-process keyframe images and poses by deleting blurry and overexposed images, and renumbering.")
    parser.add_argument("--directory", type=str, help="Directory containing the keyframe images and poses.")
    parser.add_argument("--blur-threshold", type=float, default=500.0, help="Threshold for detecting blurry images.")
    parser.add_argument("--brightness-threshold", type=float, default=200.0, help="Threshold for detecting overexposed images.")
    args = parser.parse_args()

    main(args.directory, args.blur_threshold, args.brightness_threshold)
