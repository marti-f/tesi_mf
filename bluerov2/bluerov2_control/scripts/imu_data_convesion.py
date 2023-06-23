#!/usr/bin/env python3
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist,Quaternion,Pose 
import rospy
import numpy as np
import conversion_w_to_b as conv 
import tf
import math


class IMU_data_transform:
  def __init__(self):
    rospy.init_node('IMU_data_transform')
    # ------- PARAMETERS ------- 
    self.freq = 50
    self.Tc = 1/self.freq
    #self.freq = 1/rospy.get_param('/gazebo/time_step', 0.01)
    self.rate = rospy.Rate(self.freq)
    self.v_x = 0
    self.v_y = 0 
    self.v_z = 0 
    
    self.x = 0
    self.y = 0
    self.z = 0

    self.pose_gt_x = 0
    
    # -------   TOPICS   -------
    self.pose_gt_sub = rospy.Subscriber('/bluerov2/pose_gt',Odometry,self.pose_gt_callback)
    self.vel_word_sub = rospy.Subscriber('/bluerov2/imu',Imu,self.imu_data_callback)
    self.vel_body_pub = rospy.Publisher('/bluerov2/pose_body_imu_transform',Odometry,queue_size=1)
    self.errore_pub = rospy.Publisher('/bluerov2/errore_x',Float64,queue_size=1)
  # -------   LIFE CYCLE   -------
  def loop(self):
    rospy.spin()

  # -------   CALLBACK   -------
  def pose_gt_callback(self,msg):
    self.pose_gt = msg
    self.pose_gt_x = msg.twist.twist.linear.x 

  def imu_data_callback(self, msg):

    ## Conversione da quaternioni in angoli di eulero 
    q =[msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w]
    euler = tf.transformations.euler_from_quaternion(q)
    [roll,pitch,yaw] = [euler[0],euler[1],euler[2]]

    ## Velocita` angolari
    omega_x = msg.angular_velocity.x
    omega_y = msg.angular_velocity.y 
    omega_z = msg.angular_velocity.z

    ## Compensazione della gravita`
    gx = 2*(q[1] * q[3] - q[0] * q[2])
    gy = 2*(q[0] * q[1] + q[2] * q[3])
    gz = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]

    ## Accelerazioni lineari
    a_x = msg.linear_acceleration.x - gx*9.79999998871575
    a_y = msg.linear_acceleration.y - gy*9.79999998871575
    a_z = msg.linear_acceleration.z - gz*9.79999998871575

    # Integro le accelerazioni
    self.v_x = self.v_x + a_x*self.Tc
    self.v_y = self.v_y + a_y*self.Tc
    self.v_z = self.v_z + a_z*self.Tc
    
    # Integro le velocita`
    self.x = self.x + self.v_x*self.Tc
    self.y = self.y + self.v_y*self.Tc
    self.z = self.z + self.v_z*self.Tc
    vel_body = Odometry()
    
    # header
    #vel_body.header.stamp.secs = self.pose_gt.header.stamp.secs
    #vel_body.header.stamp.nsecs = self.pose_gt.header.stamp.nsecs
    #vel_body.header.seq = self.pose_gt.header.seq
    vel_body.header.frame_id = "world"
    vel_body.child_frame_id = "bluerov2/base_link"

    vel_body.pose.pose.position.x = self.x
    vel_body.pose.pose.position.y = self.y
    vel_body.pose.pose.position.z = self.z
  
    vel_body.pose.pose.orientation = msg.orientation

    vel_body.twist.twist.linear.x = self.v_x
    vel_body.twist.twist.linear.y = self.v_y
    vel_body.twist.twist.linear.z = self.v_z

    vel_body.twist.twist.angular.x = omega_x
    vel_body.twist.twist.angular.y = omega_y
    vel_body.twist.twist.angular.z = omega_z

    errore_x = self.pose_gt_x-self.v_x
    
    # Pubblicazione sul topic 
    self.vel_body_pub.publish(vel_body)
    self.errore_pub.publish(errore_x)

# -------   INITIALIZE INSTANCE  -------
if __name__ == '__main__':
  node = IMU_data_transform()
  node.loop()