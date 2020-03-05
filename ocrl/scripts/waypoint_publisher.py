#!/usr/bin/env python

import numpy as np

import rospy
from geometry_msgs.msg import PoseArray, Pose, Quaternion
from tf.transformations import quaternion_from_euler

x_lim = [-10, 10]
y_lim = [-10, 10]
theta_lim = [-np.pi, np.pi]
num_waypoints = 10


if __name__ == '__main__':

  rospy.init_node('waypoint_publisher')

  waypoints = np.random.rand(num_waypoints, 3)
  waypoints[:, 0] = (x_lim[1] - x_lim[0]) * waypoints[:, 0] + x_lim[0]
  waypoints[:, 1] = (y_lim[1] - y_lim[0]) * waypoints[:, 1] + y_lim[0]
  waypoints[:, 2] = (theta_lim[1] - theta_lim[0]) * waypoints[:, 2] + theta_lim[0]

  waypoints_pub = rospy.Publisher('waypoints', PoseArray, queue_size=10)

  waypoint_array = PoseArray()
  for i in range(0, num_waypoints):
    w = Pose()
    w.position.x = waypoints[i, 0]
    w.position.y = waypoints[i, 1]
    w.orientation = Quaternion(*quaternion_from_euler(0, 0, waypoints[i, 2]))

    waypoint_array.poses.append(w)

  r = rospy.Rate(1)  # 10hz
  while not rospy.is_shutdown():
    waypoints_pub.publish(waypoint_array)
    r.sleep()



