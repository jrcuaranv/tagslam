# Tagslam
This project is intended for robot localization fusing visual odometry and apriltags observations.

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
1. Configure parameters in gtsam_params.yaml and stereo_cam.yaml
2. Launch tagslam.launch
```
roslaunch tagslam tagslam.launch
```