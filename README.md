<h1 align="center">
🤖 Lightweight Online 3D Mapping
</h1>
<p align="center">
Tech Stack: ROS, C++
</p>

<p align="center">
   <a href="https://github.com/senorMk/ROS-3DMapping/blob/master/LICENSE">
      <img src="https://img.shields.io/badge/License-MIT-green.svg" />
   </a>
</p>

The goal of this project was to design a mobile robot system that could generate a 3D map of an environment needed for autonomous movement.

> Developed C++ software that run on a Raspberry Pi, Arduino UNO, Ubuntu Desktop and utilized an ASUS Xtion Pro RGB-D sensor to generate the 3D maps needed for movement.

> Successfully developed a prototype that met the goals of the project.

## clone or download

```terminal
$ git clone https://github.com/senorMk/ROS-3DMapping.git
```

## project structure

```terminal
LICENSE
Arduino/
Raspberry Pi/
Laptop/
...
```

# Building

TODO

# Run Commands

Assuming that our ROS Master is running on a laptop with IP: 192.168.1.166 and port 11311. The Raspberry Pi has IP: 192.168.1.2. We're using tmux for persistent sessions wherever possible (SSH over WiFi is extremely unreliable).

## Laptop - TAB 1 - Get Roscore started

```terminal
$ export ROS_MASTER_URI=http://192.168.1.166:11311
$ export ROS_IP=192.168.1.166
$ roscore
```

## Laptop - TAB 2 - Start the Laptop Controller

```terminal
$ cd ~/Dev/Projects/Controller_Laptop
$ catkin build
$ export ROS_MASTER_URI=http://192.168.1.166:11311
$ export ROS_IP=192.168.1.166
$ rosrun controller_laptop mapping_node
```

## Raspberry Pi - TAB 1 - Start the Raspberry Pi Controller

```terminal
$ tmux new -s pi_controller
$ cd ~/Projects/Controller_PPi/src
$ catkin build
$ export ROS_MASTER_URI=http://192.168.1.166:11311
$ export ROS_IP=192.168.1.2
$ rosrun controller_rpi node
```

## Raspberry Pi - TAB 2 - Start the Serial Server

```terminal
$ tmux new -s serial_server
$ export ROS_MASTER_URI=http://192.168.1.166:11311
$ export ROS_IP=192.168.1.2
$ rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0
```

## Raspberry Pi - TAB 3 - Start the openni script - LibOpenni driver 1

```terminal
$ tmux new -s driver_1
$ export ROS_MASTER_URI=http://192.168.1.166:11311
$ export ROS_IP=192.168.1.2
$ roslaunch openni_launch openni.launch depth_registration:=true data_skip:=5
```

## Raspberry Pi - TAB 4 - Start the openni script - LibOpenni driver 2

Note: To delete the old map on start add the flag: --delete_db_on_start.

Note: To adjust the map max size set the flag: --OdomF2M/MaxSize 1000.

```terminal
$ tmux new -s driver_2
$ export ROS_MASTER_URI=http://192.168.1.166:11311
$ export ROS_IP=192.168.1.2
$ roslaunch rtabmap_ros rgbd_mapping.launch rtabmap_args:="--delete_db_on_start --Rtabmap/DetectionRate 17 --Kp/MaxFeatures 1000 --Vis/MaxFeatures 1000 --Mem/ImagePreDecimation 2 --Mem/ImagePostDecimation 2 --Kp/DetectorStrategy 6 --Odom/ImageDecimation 2 --OdomF2M/MaxSize 1000" rtabmapviz:=false
```

## Laptop - TAB 3 - Start up RVIZ

```terminal
$ export ROS_MASTER_URI=http://192.168.1.166:11311
$ export ROS_IP=192.168.1.166
$ ROS_NAMESPACE=rtabmap rosrun rtabmap_ros rtabmapviz _subscribe_odom_info:=false _frame_id:=camera_link
```
