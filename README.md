# Tagslam
This project is intended for robot localization fusing visual odometry and apriltags observations.
Currently, it only supports stereo images.
Tested in ubuntu 18.04 and Ubuntu 20.04.

## Requirements
1. ROS Noetic
2. GTSAM 4.2
3. Apriltag library
## Installation
1. Install dependencies
```
sudo apt-add-repository ppa:borglab/gtsam-release-4.2
sudo apt update
sudo apt install libgtsam-dev libgtsam-unstable-dev
pip install apriltag
```
2. Clone repository in your catkin workspace
```
cd catkin_ws/src
git clone https://github.com/jrcuaranv/tagslam.git
cd ..
catkin_make
source devel/setup.bash
```
## Running Tagslam
1. Generate an apriltags.csv file with the ground truth poses of the markers in format x, y, z, rx, ry, rz. You can use the [Tagslam project](https://berndpfrommer.github.io/tagslam_web/) to get the tags poses.
2. Configure parameters in gtsam_params.yaml and stereo_cam.yaml
3. Launch tagslam.launch
```
roslaunch tagslam tagslam.launch
```