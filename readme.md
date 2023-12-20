# UR5_Pybullet_MoveIt 
## Introduction
A UR5 robotic arm project, simulated in Pybullet, and implemented communication between Pybullet and ROS MoveIt, and implemented robotic arm planning through MoveIt.

## Dependencies
### pybullet
```sh
pip install pybullet
```
### gin-config
```sh
pip install gin-config
```
### python3-empy
```sh
sudo apt-get install python3-empy
```
## Build
 in the ros workspace, run the following command in the terminal:
 ```sh
catkin build
 ```

## Setup
 Run the following command in the terminal to edit your ~/.bashrc
``` sh
gedit ~/.bashrc
```
add the following command at the end of your ~/.bashrc file.
``` sh  [the absolute path of your ros workspace]/devel/setup.bash
source 
```
where the [the absolute path of your ros workspace] is the the absolute path of your ros workspace. You can get this path by running command *pwd* in yout terminal open at the folder of the ros workspace.
## Run
run the following command in the terminal:
```sh
roslaunch ur5_pybullet_ros ur5_pybullet_ros_env.launch 

```
Then you can see the pybullet interface and rviz are launched. You can directly see the `TF`frames in the rviz windows. You can also see all the `TF`frames by running `rosrun rqt_tf_tree rqt_tf_tree` in your terminal.
