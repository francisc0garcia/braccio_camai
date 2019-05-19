# braccio_driver

# Description

This driver integrates ```Braccio robot arm``` into ROS, using ```move_it``` for motion planning.

It is assumed that the robot arm is connected to an arduino-compatible board (tested using Arduino Leonardo and Genuino 101). Firmware is available in ```arduino``` folder.

# Usage

## 0. Install dependencies

First install ROS (tested on Kinetic and Melodic), and additional dependencies:

```sudo apt install ros-$ROS_DISTRO-robot-state-publisher ros-$ROS_DISTRO-joint-state-publisher ros-$ROS_DISTRO-moveit-ros-move-group ros-$ROS_DISTRO-moveit-fake-controller-manager ros-$ROS_DISTRO-moveit-planners ros-$ROS_DISTRO-moveit-planners-ompl ros-$ROS_DISTRO-moveit-ros-planning```

Additional (optional) dependencies:

```sudo apt install python3-opencv python3-pip python3-yaml```

```sudo pip3 install rospkg catkin_pkg```

## 1. Get and install the project

Clone the project into yout ROS workspace:

```cd your_ros_workspace/src```

```git clone https://github.com/francisc0garcia/braccio_driver```

Build the project

```cd braccio_driver && catkin build --this```

## 2. Install arduino firmware

Using arduino IDE, install the firmware located in folder ```arduino/BraccioRobotRos```, notice that library ```BraccioRobot``` must be installed before using this firmware.
 By default, once the robot arm is powered, it will move to the start position (straight up).

## 3. Control robot arm using RVIZ

This project constains two main launch files:

- ```braccio_driver_moveit.launch```: Execute the driver and ```move it``` pipeline

- ```braccio_gui.launch```: Start graphical interface

Optional: It is strongly recommended to install and use [fkie_node_manger](https://github.com/fkie/multimaster_fkie), because it simplifies the interaction with ROS.

If not, it is also possible to run the nodes manually:

```roslaunch braccio_driver braccio_driver_moveit.launch```

```roslaunch braccio_driver braccio_gui.launch```

# Author

Francisco J. Garcia R.

2019

# License

This project is licensed under MIT.

This project was based on previous developments: [ros_braccio_urdf](https://github.com/grassjelly/ros_braccio_urdf) and [ros_braccio_moveit](https://github.com/zakizadeh/ros_braccio_moveit)