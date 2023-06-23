#!/usr/bin/env python3
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist,Quaternion,Pose 
import rospy
import numpy as np
import conversion_w_to_b as conv 
import tf
import math


class world_to_body_transform:
  def __init__(self):
    rospy.init_node('world_to_body')
    # ------- PARAMETERS ------- 
    self.freq = 1/0.001
    #self.freq = 1/rospy.get_param('/gazebo/time_step', 0.001)
    self.rate = rospy.Rate(self.freq)

    # -------   TOPICS   -------
    self.vel_word_sub = rospy.Subscriber('/bluerov2/pose_gt',Odometry,self.vel_world_callback)
    self.vel_body_pub = rospy.Publisher('/bluerov2/pose_body',Odometry,queue_size=1)

  # -------   LIFE CYCLE   -------
  def loop(self):
    rospy.spin()

  # -------   CALLBACK   -------
  def vel_world_callback(self, msg):
    vel_body = conv.ConversionW2B(msg)

    # Pubblicazione sul topic 
    self.vel_body_pub.publish(vel_body)

# -------   INITIALIZE INSTANCE  -------
if __name__ == '__main__':
  node = world_to_body_transform()
  node.loop()