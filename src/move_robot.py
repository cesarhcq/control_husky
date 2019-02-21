#!/usr/bin/env python
from __future__ import print_function
import rospy
import time, math
import sys, select, termios, tty
import numpy as np
from math import atan2, sin, cos, pi
from nav_msgs.msg import Odometry, Path
#from tf.transformations import euler_from_quaternion
from rosgraph_msgs.msg import Clock 
from std_msgs.msg import String
from geometry_msgs.msg import Point, Twist, PoseStamped
import roslib; roslib.load_manifest('control_husky_teleop')
contador = 0

amplitude1 = 0.5
amplitude2 = 0.9
Tau = 2
omega = 2*3.1415/Tau
t_ini = 1

husky_pose = Odometry()
path_husky = Path()
pose = PoseStamped()

msg = """
Keyboard commands for Husky Robot
----------------------------------------------------------
Move para Frente     - Press 1
Rotacao Direita      - Press 2
Rotacao Esquerda      - Press 3
----------------------------------------------------------
Ctrl + C to quit
"""
def getKey():
  tty.setraw(sys.stdin.fileno())
  select.select([sys.stdin], [], [], 0)
  key = sys.stdin.read(1)
  termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
  return key

#Funcao para mover robo para frente
def move_forward():
  global contador, amplitude, omega

  velocity = Twist()

  contador = 0
  

  while contador < 1000:

    print('contador frente: ', contador)
    velocity.linear.x = abs(amplitude1*(sin(omega*rospy.get_time())))
    velocity.linear.y = 0.0
    velocity.linear.z = 0.0
    velocity.angular.x = 0
    velocity.angular.y = 0
    velocity.angular.z = 0
    pose_pub.publish(velocity)
    contador += 1
    print('vel_x: {} ang_z: {}'.format(velocity.linear.x, velocity.angular.z))
    #print('vel_x: {} - vel_y: {} - vel_z: {}'.format(velocity.linear.x, velocity.linear.y, velocity.linear.z))
    rate.sleep()

def rotate_left():
  global contador
  velocity = Twist()  
  contador = 0

  while contador < 100:

    print('contador rotacao: ', contador)
    velocity.angular.x = 0
    velocity.angular.y = 0
    velocity.angular.z = 3.14159265359/2
    pose_pub.publish(velocity)

    contador += 1
    print('ang_z: {}'.format(velocity.angular.z))
    rate.sleep()

def rotate_right():
  global contador
  velocity = Twist()  
  contador = 0

  while contador < 100:

    print('contador rotacao: ', contador)
    velocity.angular.x = 0
    velocity.angular.y = 0
    velocity.angular.z = -2
    pose_pub.publish(velocity)

    contador += 1
    print('ang_z: {}'.format(velocity.angular.z))
    rate.sleep()

def callbackPathGeneration(posedata):
  global path_husky, husky_pose

  husky_pose.header = posedata.header
  husky_pose.pose = posedata.pose

  pose.header = posedata.header
  pose.header.stamp = rospy.Time.now()
  pose.pose = posedata.pose.pose

  path_husky.header = pose.header
  path_husky.poses.append(pose)

  path_pub.publish(path_husky)



if __name__ == '__main__':
  settings = termios.tcgetattr(sys.stdin)
  rospy.init_node('control_husky_teleop')
  path_sub = rospy.Subscriber("/imu/data", Odometry, callbackPathGeneration)
  # create the important publishers
  pose_pub = rospy.Publisher("/cmd_vel",Twist, queue_size=100)
  path_pub = rospy.Publisher("/husky_pose/groundtruth_husky", Path, queue_size = 100)

  rate = rospy.Rate(100) #-- 100Hz

  print('Program Started')
  print(msg)

  try:
    while(1):
      key = getKey()
      print('key')  # add a print for each key pressed
      print(key)

      if key == '1': # condition created in order to pressed key 1 and generates the take off of the bebop2
        print('key 1 pressed - Move frente')
        move_forward() # action to publish it
        print(msg)

      elif key == '2': # condition created in order to pressed key 2 and generates the land of the bebop2
        print('key 2 pressed - Rotacao left')
        rotate_left() # action to publish it
        print(msg)

      elif key == '3': # condition created in order to pressed key 2 and generates the land of the bebop2
        print('key 3 pressed - Rotacao right')
        rotate_right() # action to publish it
        print(msg)

      elif key == '\x03': # condition created in order to pressed key Ctrl+c and generates output from the program
          print('Quit work')
          break
      else:
        print('Wrong key!')
        print(msg)

    
  except rospy.ROSInterruptException:
    print('Erro')