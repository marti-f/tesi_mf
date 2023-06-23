#!/usr/bin/env python3

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist,Quaternion,Pose 
import rospy
import numpy as np
import tf
import math

def ConversionW2B(pose_gt):

    pose_w = pose_gt.pose.pose
    vel_w = pose_gt.twist.twist 
    #conversione da quaternioni in angoli di eulero
    quaternion =(pose_w.orientation.x,pose_w.orientation.y,pose_w.orientation.z,pose_w.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    [roll, pitch, yaw] = [euler[0], euler[1],euler[2]]
        
    # Calcolo della matrice di rotazione dal frame body al frame world
    R = np.array([[math.cos(yaw)*math.cos(pitch), -math.sin(yaw)*math.cos(roll)+math.cos(yaw)*math.sin(pitch)*math.sin(roll), math.sin(yaw)*math.sin(roll)+math.cos(yaw)*math.cos(roll)*math.cos(pitch)],[math.sin(yaw)*math.cos(pitch), math.cos(yaw)*math.cos(roll)+math.sin(roll)*math.sin(pitch)*math.sin(yaw),- math.cos(yaw)*math.sin(roll)+math.sin(pitch)*math.sin(yaw)*math.cos(roll)],[-math.sin(pitch), math.cos(pitch)*math.sin(roll), math.cos(pitch)*math.cos(roll)]])

    # Matrice di trasformazione angolare dal frame body al frame world
    T = np.array([[1,math.sin(roll)*math.tan(pitch),math.cos(roll)*math.tan(pitch)],[0,math.cos(roll),-math.sin(roll)],[0, math.sin(roll)/math.cos(pitch),math.cos(roll)/math.cos(pitch)]])
        
    # Matrice di trasformazione delle velocita` dal frame world a body`
    R_1 = np.linalg.inv(R)
    T_1 = np.linalg.inv(T)
    # calcolo velocita` nel frame body

    # vettore delle velocita` lineari
    vel_l_world_array = np.array([vel_w.linear.x,vel_w.linear.y,vel_w.linear.z])

    # vettore delle velocita` angolari
    vel_a_world_array = np.array([vel_w.angular.x,vel_w.angular.y,vel_w.angular.z])
    vel_l_body = np.dot(R_1,vel_l_world_array)
    vel_a_body = np.dot(T_1,vel_a_world_array)

    vel_body = Odometry()
    
    vel_body.header.frame_id = "world_ned"
    vel_body.header.seq = pose_gt.header.seq
    vel_body.header.stamp.secs = pose_gt.header.stamp.secs
    vel_body.header.stamp.nsecs = pose_gt.header.stamp.nsecs
    vel_body.child_frame_id = "bluerov2/base_link"


    vel_body.twist.twist.linear.x = vel_l_body[0]
    vel_body.twist.twist.linear.y = vel_l_body[1]
    vel_body.twist.twist.linear.z = vel_l_body[2]

    vel_body.twist.twist.angular.x = vel_a_body[0]
    vel_body.twist.twist.angular.y = vel_a_body[1]
    vel_body.twist.twist.angular.z = vel_a_body[2]
    return vel_body