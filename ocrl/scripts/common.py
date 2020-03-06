#!/usr/bin/env python

import numpy as np
import math

import rospy
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point, PoseArray
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

x_lim = [-10, 10]
y_lim = [-10, 10]
theta_lim = [-np.pi, np.pi]
num_waypoints = 10
waypoint_tol = 0.2

wheelbase = 0.335
max_acc = 3

