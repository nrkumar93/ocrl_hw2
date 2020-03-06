#!/usr/bin/env python

from common import *
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, Pose, Twist
from ackermann_msgs.msg import AckermannDriveStamped
from angles import *

import tf

def waypointCallback(msg):
  global waypoints
  for i in range(len(msg.poses)):
    waypoints[i, 0] = msg.poses[i].position.x
    waypoints[i, 1] = msg.poses[i].position.y
    waypoints[i, 2] = euler_from_quaternion([msg.poses[i].orientation.x, msg.poses[i].orientation.y, msg.poses[i].orientation.z, msg.poses[i].orientation.w])[2]

def vehicleStateCallback(msg):
  global rear_axle_center, rear_axle_theta, rear_axle_velocity
  rear_axle_center.position.x = msg.pose.pose.position.x
  rear_axle_center.position.y = msg.pose.pose.position.y
  rear_axle_center.orientation = msg.pose.pose.orientation

  rear_axle_theta = euler_from_quaternion(
    [rear_axle_center.orientation.x, rear_axle_center.orientation.y, rear_axle_center.orientation.z,
     rear_axle_center.orientation.w])[2]

  rear_axle_velocity.linear = msg.twist.twist.linear
  rear_axle_velocity.angular = msg.twist.twist.angular

def pursuitToWaypoint(waypoint):
  print waypoint
  global rear_axle_center, rear_axle_theta, rear_axle_velocity, cmd_pub
  dx = waypoint[0] - rear_axle_center.position.x
  dy = waypoint[1] - rear_axle_center.position.y
  # beta = shortest_angular_distance(waypoint[2], rear_axle_theta)
  # lookahead_dist = np.sqrt(dx*dx + dy*dy)
  # lookahead_theta = math.atan2(dy, dx)
  # alpha = shortest_angular_distance(waypoint[2], lookahead_theta)
  # # target_distance = math.sqrt(dx*dx + dy*dy + beta*beta)
  target_distance = math.sqrt(dx*dx + dy*dy)

  cmd = AckermannDriveStamped()
  cmd.header.stamp = rospy.Time.now()
  cmd.header.frame_id = "base_link"
  cmd.drive.speed = rear_axle_velocity.linear.x
  cmd.drive.acceleration = max_acc
  while target_distance > waypoint_tol:

    dx = waypoint[0] - rear_axle_center.position.x
    dy = waypoint[1] - rear_axle_center.position.y
    lookahead_dist = np.sqrt(dx * dx + dy * dy)
    lookahead_theta = math.atan2(dy, dx)
    alpha = shortest_angular_distance(waypoint[2], lookahead_theta)

    cmd.header.stamp = rospy.Time.now()
    cmd.header.frame_id = "base_link"
    # cmd.drive.speed = rear_axle_velocity.linear.x
    cmd.drive.speed = 1
    # cmd.drive.acceleration = -2
    # cmd.drive.acceleration = max_acc
    # if abs(rear_axle_velocity.linear.x) < 5:
    #   R = lookahead_dist / math.sin(alpha) * 0.5
    #   cmd.drive.steering_angle = math.atan(2.0*wheelbase*math.sin(alpha)/lookahead_dist)
    # else:
    #   cmd.drive.steering_angle += alpha
    cmd.drive.steering_angle = math.atan(2.0*wheelbase*math.sin(alpha)/lookahead_dist)
    # cmd.drive.steering_angle += alpha

    # beta = shortest_angular_distance(waypoint[2], rear_axle_theta)
    # target_distance = math.sqrt(dx * dx + dy * dy + beta * beta)
    target_distance = math.sqrt(dx * dx + dy * dy)

    cmd_pub.publish(cmd)
    rospy.wait_for_message("/ackermann_vehicle/ground_truth/state", Odometry, 5)


if __name__ == '__main__':

  rospy.init_node('pure_pursuit')
  cmd_pub = rospy.Publisher('/ackermann_vehicle/ackermann_cmd', AckermannDriveStamped, queue_size=10)

  waypoints = np.zeros((num_waypoints, 3))
  rospy.Subscriber("/ackermann_vehicle/waypoints",
                   PoseArray,
                   waypointCallback)
  rospy.wait_for_message("/ackermann_vehicle/waypoints", PoseArray, 5)


  rear_axle_center = Pose()
  rear_axle_velocity = Twist()
  rospy.Subscriber("/ackermann_vehicle/ground_truth/state",
                   Odometry, vehicleStateCallback)
  rospy.wait_for_message("/ackermann_vehicle/ground_truth/state", Odometry, 5)

  for w in waypoints:
    pursuitToWaypoint(w)




