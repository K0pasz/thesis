# Laptop mapping visualization and keyframe collection for Gaussian Splatting

## Dependencies

- `sudo apt update && sudo apt upgrade -y`
- ROS2 Humble framework: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
- COLMAP, FFMPEG and NVIDIA driver for Nerfstudio:
  - COLMAP: https://colmap.github.io/install.html#debian-ubuntu
  - FFMPEG: `sudo apt install ffmpeg`
  - NVIDIA driver: `sudo ubuntu-drivers install` (if it does not work, run `sudo ubuntu-drivers list` and `sudo ubuntu-drivers install nvidia:SELECT_VERSION_FROM_LIST`)
- Nerfstudio: https://docs.nerf.studio/quickstart/installation.html

## Build

```
colcon build
source install/setup.bash
```

## Usage

### Visualizing the mapping

First start the node on the Turtlebot.

For visualizing the mapping, run the following command:
```
ros2 launch src/rviz2_laptop/launch/mapping_rviz.py
```
Now you should see the published point cloud, the position and orientation of the camera and the published images from the camera in RViz.

### Collecting keyframes for Gaussian Splatting

For collecting the keyframes, run the following command inside another terminal window:
```
ros2 run keyframe_saver keyframe_saver
```
The keyframes will be saved in the `keyframes/` folder, which is created in the folder where you started the node.

After the mapping is finished, you can stop the node with `Ctrl + C`.

### Post processing keyframes

The post processing is done by the `post_process_images.py` script. Run it with the following command:
```
python post_process_images.py
```
It will delete the blurry and overexposed images and renumber them.

### Generating photorealistic 3D models

To generate photorealistic 3D models, first we need to convert the keyframes into a COLMAP database. Run the following command:
```
ns-process-data images --data keyframes/ --output-dir gsplat_input/
```
It should take a few minutes for generating the COLMAP.

After it has finished, run the following command to create a photorealistic 3D model using Gaussian Splatting:
```
ns-train splatfacto --data gsplat_input/
```
OR
```
ns-train splatfacto-big --data gsplat_input/
```
for a bigger model that will take more time to train but the result will be more accurate.

The output should be in the `outputs/` folder.

TODO: add the export command for the 3D model from the Ubuntu laptop.
