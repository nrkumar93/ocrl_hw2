16-745 Optimal Control and Reinformcement Learning : Homework 2 
==================================================

The goal of the assignment is to generate a local policy that drives an Ackermann system to a sequence of waypoints `(x, y, theta)` in the shortest time possible. The homework is open-ended and there is no restriction on the type of Optimal Control/RL/Planning/Learning methods you can use to achieve this task. 

## Software
This homework requrires setting up Robot Operating System (ROS) and Gazebo environments. 

### Requriements
- Ubuntu 16.04 o later
- ROS Kinetic or later
- Gazebo 7 or later

### Setting up the workspace
Get this repository in a catkin workspace. The suggested location is `~/catkin_ws/src/`, but any valid catkin worskspace source folder will work. We suggest forking over and cloning as you will be working with the code.

### Compilation and Running
```
cd ~/catkin_ws
catkin_make
```
_Note:_ If you are unfamiliar with catkin, please know that you must run `source ~/catkin_ws/devel/setup.sh` before ROS will be able to locate the ocrl packages. This line can be added to your `~/.bashrc` file so that it is automatically run on opening a terminal. 

### Testing the Simulation and Visualization
```
roslaunch ocrl ocrl.launch
rosrun ocrl pure_pursuit.py

```








