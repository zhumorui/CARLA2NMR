# CARLA2NMR Viewer

CARLA2NMR Viewer is an application designed to support the visualization of data in COLMAP format, including LiDAR point clouds and camera poses. This viewer application also supports cropped LiDAR point clouds (based on camera POV), Gaussian SLAM (in progress), and LiDAR odometry using KISS-ICP.

## Features

- **COLMAP Format Support**: Load and visualize data in COLMAP format.
- **LiDAR Point Cloud Visualization**: Display full LiDAR point clouds as well as cropped point clouds based on camera POV.
- **Camera Pose Visualization**: Show camera poses from the loaded data.
- **Gaussian SLAM**: In-progress feature to support Gaussian SLAM.
- **KISS-ICP LiDAR Odometry**: LiDAR odometry using KISS-ICP.

## Usage
1. Clone the repository:
```bash
git clone https://github.com/zhumorui/CARLA2NMR.git
```
2. Navigate to the repository directory:
```bash
cd CARLA2NMR
```
3. Installation:
```bash
pip install -r requirements.txt
```
4. Run the app:
```bash
python src/main.py
```

## Demo
1. Lidar odometry with kiss-icp method.
<p align="center">
  <img src="assets/kiss-icp_vis.gif" alt="Lidar odometry with kiss-icp" >
</p>
2. Visualize images and camera poses. 
<p align="center">
  <img src="assets/poses_vis.gif" alt="Visualize images and camera poses" >
</p>