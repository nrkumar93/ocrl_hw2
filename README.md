16-745 Optimal Control and Reinformcement Learning : Homework 2 
==================================================

The goal of the assignment is to generate a local policy that drives an Ackermann system to a sequence of waypoints `(x, y, theta)` in the shortest time possible while respecting the dynamics (non-holonomic) and control saturation (velocity, acceleration and steering angle limits). The homework is open-ended and there is no restriction on the type of Optimal Control/RL/Planning/Learning methods you can use to achieve this task. 

## Software
This homework requrires setting up Robot Operating System (ROS) and Gazebo environments. 

### Requriements
- Ubuntu 14.04 or later
- ROS Indigo or later
- Gazebo 7 or later

### Setting up the workspace
Get this repository in a catkin workspace. The suggested location is `~/catkin_ws/src/`, but any valid catkin worskspace source folder will work. We suggest forking over and cloning as you will be working with the code.

#### Additional requirements
The following ROS packages might need to be installed. 
```
effort_controllers
joint_state_publisher
``` 
The above ROS packages can be installed with the below command
```
sudo apt-get install ros-<ros_distro_name>-<package_name>
```
In case any other ROS packages are missing use the above command to install it. 

### Compilation and Running
```
cd ~/catkin_ws
catkin_make
```
_Note:_ If you are unfamiliar with catkin, please know that you must run `source ~/catkin_ws/devel/setup.sh` before ROS will be able to locate the ocrl packages. This line can be added to your `~/.bashrc` file so that it is automatically run on opening a terminal. 

### Testing the Simulation and Visualization
Run the following launch file to invoke the simulator, visualization and ordered set of waypoints to navigate to in the shortest time. Running this launch file will be first step of the planner you are going to develop
```
roslaunch ocrl ocrl.launch
```
Running the above command should open the following RViz window with a set of randomly chosen waypoints
![Sample task](img/env_rviz_layout.png)


Once the environment is ready, you can launch your planner. As an example, we have provided a pure pursuit planner/controller (below command) that drives to the set of ordered waypoints. Note that the behavior is very reactive and it does a pretty bad job in reaching all those waypoints. **You guys should be able to design a planner that does much better than this!!!**
```
rosrun ocrl pure_pursuit.py
```

### Question





### What to turn in?

You can use C++/Python/Matlab for this assignment. You have . You should use Matlab graphics to draw pictures of your robot and the obstacles, that allows rotation of the picture so it can be viewed from any angle (modify inverse kinematics class examples and use plot3()). You can work in groups or alone. Generate a web page describing what you did (one per group). Include links to your source and any compiled code in either .zip, .tar, or .tar.gz format. Be sure to list the names of all the members of your group. Mail the URL of your web page to cga@cmu.edu and Ramkumar Natarajan rnataraj@cs.cmu.edu. The writeup is more important than the code. What did you do? Why did it work? What didn't work and why?












