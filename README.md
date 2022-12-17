![Build Status](https://github.com/guruadp/inspection-robot/actions/workflows/build_and_coveralls.yml/badge.svg)
[![Coverage Status](https://coveralls.io/repos/github/guruadp/inspection-robot/badge.svg?branch=master)](https://coveralls.io/github/guruadp/inspection-robot?branch=master)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)

# inspection-robot

## Overview and Description
This is the final Project for the Software Development for Robotics (ENPM808X) course. Turtlebot3 is navigated in a warehouse to desired locations where items are placed. Each item is identified by the robot using a camera integrated with the robot that detects the ARUCO marker present in the box. Using this the classification of items is performed, and a report is generated to account for the available and the misplaced items.

# Author
## Phase  1
- Dhanush Babu Allam (Driver)
- Guru Nandhan A D P (Navigator)
- Vignesh RR (Design Keeper)

## Phase 2
- Guru Nandhan A D P (Driver)
- Vignesh RR (Navigator)
- Dhanush Babu Allam (Design Keeper)

## Phase 3
- Vignesh RR (Driver)
- Dhanush Babu Allam (Navigator)
- Guru Nandhan A D P (Design Keeper)

# Sprint
https://docs.google.com/spreadsheets/d/1A0Pc2_M6MppBpWuW0mJaRoHsq_wDIXbxwaVrvBabY-E/edit?usp=sharing

# Video
## Phase 1 video
https://drive.google.com/file/d/1sNx6R_Ter5Cd_EBcwtlR65Pktl6iZPkA/view?usp=sharing

# Presentation
## Phase 3 Video
https://docs.google.com/presentation/d/1EJEmD1B4Bu8JoqJP8fsrnUJ7H9YMx_E5Tci192NI_EE/edit?usp=sharing

# Tools, libraries and dependencies used and its installation steps
- C++ 14
- ROS Galactic
```
sudo apt update
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) \
  signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu \
  $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-galactic-ros-base python3-colcon-common-extensions
```
- OpenCV
```
sudo apt install -y g++ cmake make git libgtk2.0-dev pkg-config
git clone https://github.com/opencv/opencv.git
mkdir -p build && cd build
cmake ../opencv
make -j4
sudo make install
```
- CMake
- Git
```
sudo apt-get install git-all
```
- Github CI
- Coverall
```
sudo apt install -y lcov
```
- Visual Studio Code (https://code.visualstudio.com/download)

- RViz
- Gazebo
- turtlebot3
- ROStest, GTest, GMock


## Phase 2 progress

## Turtlebot3

Install Turtlbot3
```
cd ~/<workspace_name>/src/
git clone -b galactic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd  ..
colcon build
```

Open bashrc file
```
cd gedit ~/.bashrc
```
Add the below command in .bashrc file
```
export TURTLEBOT3_MODEL=waffle_pi
```

To check
```
ros2 launch turtlebot3_gazebo turtlebot3_world.launch
```

## Run project

Setting up workspace and build package
```
mkdir -p <workspace_name>/src
cd <workspace_name>/src
git clone https://github.com/guruadp/inspection-robot.git
source /opt/ros/galactic/setup.bash
cd ..
colcon build --packages-select inspection-robot
. install/setup.bash
```

Running package
```
ros2 launch inspection-robot inspection.launch.py
```

Open another terminal
```
source /opt/ros/galactic/setup.bash
ros2 run inspection-robot moving_robot
```

## Building for test coverage

```
cd <workspace_name>
rm -rf build/inspection-robot/
colcon build --cmake-args -DCOVERAGE=1 --packages-select inspection-robot
cat log/latest_build/inspection-robot/stdout_stderr.log
```

## Run test
```
colcon test --packages-select inspection-robot
cat log/latest_test/inspection-robot/stdout_stderr.log
```

## Generate code coverage report
```
ros2 run inspection-robot generate_coverage_report.bash
```

To open the codecoverage report

```
cd <workspace_name>
firefox /install/inspection-robot/coverage/index.html
```