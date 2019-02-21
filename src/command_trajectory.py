#!/usr/bin/env python

import rospy
from math import atan2
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
import roslib; roslib.load_manifest('control_husky_teleop')

x = 0.0
y = 0.0
theta = 0.0

def newOdom(msg):

	global x, y, theta

	x = msg.pose.pose.position.x
	y = msg.pose.pose.position.y

	rot_q = msg.pose.pose.orientation
	(roll, pitch, theta) = euler_from_quaternion ([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
	print('theta:{}'.format(theta))

rospy.init_node("control_husky_teleop")

sub = rospy.Subscriber('/odometry/filtered', Odometry, newOdom)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)

speed = Twist()

r = rospy.Rate(4)

goal = Point()
goal.x = 5.0
goal.y = 0.0

while not rospy.is_shutdown():
	inc_x = goal.x - x
	inc_y = goal.y - y

	angle_to_goal = atan2 (inc_y, inc_x)
	#print('angulo:{}'.format(angle_to_goal))

	if abs(angle_to_goal - theta) > 0.1:
		speed.linear.x = 0.3
		speed.angular.z = 0.3
	else:
		speed.linear.x = 0.5
		speed.angular.z = 0.0

	pub.publish(speed)
	r.sleep()
