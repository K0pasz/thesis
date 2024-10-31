import cv2
import os
import glob
import argparse
import numpy as np

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
    """Delete images that are blurry or overexposed."""
    images = sorted(glob.glob(os.path.join(directory, "*.png")))
    for image_path in images:
        if is_blurry(image_path, blur_threshold) or is_overexposed(image_path, brightness_threshold):
            print(f"Deleting blurry or overexposed image: {image_path}")
            os.remove(image_path)

def renumber_images(directory):
    """Renumber images in the directory sequentially after deletion."""
    images = sorted(glob.glob(os.path.join(directory, "*.png")))
    for i, image_path in enumerate(images):
        new_name = os.path.join(directory, f"keyframe_{i:04d}.png")
        if image_path != new_name:
            os.rename(image_path, new_name)
            print(f"Renaming {image_path} to {new_name}")

def main(directory, blur_threshold, brightness_threshold):
    """Run the post-processing on the specified directory."""
    delete_blurry_and_overexposed_images(directory, blur_threshold, brightness_threshold)
    renumber_images(directory)
    print("Post-processing completed.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Post-process keyframe images by deleting blurry and overexposed images, and renumbering.")
    parser.add_argument("--directory", type=str, help="Directory containing the keyframe images.")
    parser.add_argument("--blur-threshold", type=float, default=500.0, help="Threshold for detecting blurry images.")
    parser.add_argument("--brightness-threshold", type=float, default=200.0, help="Threshold for detecting overexposed images.")
    args = parser.parse_args()

    main(args.directory, args.blur_threshold, args.brightness_threshold)
