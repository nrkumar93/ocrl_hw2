#!/usr/bin/env python

from common import *
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, Pose
from ackermann_msgs.msg import AckermannDriveStamped

import tf

def waypointCallback(msg):
  global waypoints
  for i in range(len(msg.poses)):
    waypoints[i, 0] = msg.poses[i].position.x
    waypoints[i, 1] = msg.poses[i].position.y
    waypoints[i, 2] = euler_from_quaternion([msg.poses[i].orientation.x, msg.poses[i].orientation.y, msg.poses[i].orientation.z, msg.poses[i].orientation.w])[2]

def vehicleStateCallback(msg):
  global rear_axle_center, rear_axle_theta
  s = Odometry()
  rear_axle_center.position.x = s.pose.pose.position.x
  rear_axle_center.position.y = s.pose.pose.position.y
  rear_axle_center.orientation = s.pose.pose.orientation

  rear_axle_theta = euler_from_quaternion(
    [rear_axle_center.orientation.x, rear_axle_center.orientation.y, rear_axle_center.orientation.z,
     rear_axle_center.orientation.w])[2]

def pursuitToWaypoint(waypoint):
  global rear_axle_center, rear_axle_theta
  dx = waypoint[0] - rear_axle_center.position.x
  dy = waypoint[1] - rear_axle_center.position.y
  lookahead_dist = np.sqrt(dx*dx + dy*dy)
  lookahead_theta = math.atan2(dy, dx)


if __name__ == '__main__':

  rospy.init_node('pure_pursuit')
  cmd = rospy.Publisher('/ackermann_vehicle/ackermann_cmd', AckermannDriveStamped, queue_size=10)

  waypoints = np.zeros((num_waypoints, 3))
  rospy.Subscriber("/ackermann_vehicle/waypoints",
                   PoseArray,
                   waypointCallback)
  rospy.wait_for_message("/ackermann_vehicle/waypoints", PoseArray, 5)


  rear_axle_center = Pose()
  rospy.Subscriber("/ackermann_vehicle/ground_truth/state",
                   Odometry, vehicleStateCallback)
  rospy.wait_for_message("/ackermann_vehicle/ground_truth/state", Odometry, 5)




