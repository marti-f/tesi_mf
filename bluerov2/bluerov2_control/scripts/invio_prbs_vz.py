#!/usr/bin/env python3

import rospy
import yaml 
import numpy as np
from roslib.packages import get_pkg_dir
import random as rnd
from geometry_msgs.msg import Wrench
from uuv_sensor_ros_plugins_msgs.msg import DVL
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist,Quaternion,Pose 


class invio_prbs:
  def __init__(self):

    rospy.init_node('prbs')

    # ------- PARAMETERS ------- 
    self.freq = 50
    self.rate = rospy.Rate(self.freq)
    
    #INPUT
    self.fx = 0
    self.input_pub = Wrench()
    
    self.data_id = []
    self.count = 0


    # -------   TOPICS   -------    
    self.thruster_pub = rospy.Publisher('/bluerov2/thruster_manager/input',Wrench,queue_size=1)
    self.read_output_sub = rospy.Subscriber('/bluerov2/dvl',DVL,self.read_callback)
  # -------   LIFE CYCLE   -------

  def write_to_file(self,data):
    data_path = get_pkg_dir('bluerov2_control') + '/config/identificazione_vz.yaml'
    with open(data_path, 'w') as file:
        yaml.dump(data, file)
  
  def loop(self):
    while not rospy.is_shutdown():
      if len(self.data_id) == 1000:
        self.write_to_file(self.data_id)
        break

  # -------   CALLBACK   -------
  def read_callback(self,msg):
    self.fz = rnd.choice([-3100, 3100])
    self.input_pub.force.z = self.fz
    self.thruster_pub.publish(self.input_pub)
    output_data = msg.velocity.z
    input_data = self.fz
    row = [input_data,output_data]
    self.data_id.append(row)




# -------   INITIALIZE INSTANCE  -------
if __name__ == '__main__':
  node = invio_prbs()
  node.loop()