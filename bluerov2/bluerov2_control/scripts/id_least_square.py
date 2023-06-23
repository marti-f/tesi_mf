#!/usr/bin/env python3

import rospy
import yaml 
import numpy as np
import scipy.io
from roslib.packages import get_pkg_dir
import random as rnd
from geometry_msgs.msg import Wrench
from uuv_sensor_ros_plugins_msgs.msg import DVL
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist,Quaternion,Pose 


class id_LQ:
  def __init__(self):
    rospy.init_node('id_LQ')

    # ------- PARAMETERS ------- 
    self.freq = 50
    self.rate = rospy.Rate(self.freq)
    self.count = 0



    # -------   TOPICS   -------    

  # -------   LIFE CYCLE   -------
  def write_to_file(self,data,data_path):
    with open(data_path, 'w') as file:
        yaml.dump(data, file)


  def loop(self):
    while not rospy.is_shutdown():
        filepath = get_pkg_dir('bluerov2_control') + '/config/validazione/validazione_vx.yaml'
        with open(filepath, 'r') as file:
            matrix = yaml.safe_load(file)
        # input
        input=[]
        # output 
        output =[]
        for i in range(len(matrix)):
            input.append(matrix[i][0])
            output.append(matrix[i][1])
        # Identificazione
        data_path_in = get_pkg_dir('bluerov2_control') + '/config/validazione/sin_vx.yaml'
        self.write_to_file(input,data_path_in)
        data_path_out = get_pkg_dir('bluerov2_control') + '/config/validazione/vx.yaml'
        self.write_to_file(output,data_path_out)
        scipy.io.savemat('data_val_vx.mat',{'vx':output,'sin':input})
        break
  # -------   CALLBACK   -------


# -------   INITIALIZE INSTANCE  -------
if __name__ == '__main__':
  node = id_LQ()
  node.loop()
