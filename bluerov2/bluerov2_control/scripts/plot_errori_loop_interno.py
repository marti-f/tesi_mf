#!/usr/bin/env python3

import rospy
import yaml 
import numpy as np
from roslib.packages import get_pkg_dir
import random as rnd
from geometry_msgs.msg import Twist
from bluerov2_control.msg import Pipeline_State
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
from uuv_sensor_ros_plugins_msgs.msg import DVL
from geometry_msgs.msg import Wrench
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

class plot_interno:
  def __init__(self):
    rospy.init_node('plot_loop_interno')

    # ------- PARAMETERS ------- 
    self.freq = 1/0.02
    self.rate = rospy.Rate(self.freq)
    
    #INPUT
    self.data_errori = []
    self.data_vel_ref = []
    self.data_vel_meas =[]
    self.omega_meas =[]
    self.data_thruster =[]
    self.data_pose =[]
    self.count = 0
    self.row_e = []
    self.row_omega_meas = 0.0
    self.row_pose = []
    self.row_t = []
    self.row_vm = []
    self.row_vr = []

    


    # -------   TOPICS   -------    
    #self.errori_plot_sub = rospy.Subscriber('/bluerov2/estimated_state', Pipeline_State,self.estimated_state_callback)
    #self.thruster_pub = rospy.Publisher('/bluerov2/thruster_manager/input',Wrench,queue_size=1)
    #self.velo_riferimento = rospy.Subscriber('/bluerov2/cmd_vel',Twist,self.velo_callback)
    self.vel_dvl_sub = rospy.Subscriber('/bluerov2/dvl',DVL,self.readDVL)
    self.vel_imu_sub = rospy.Subscriber('/bluerov2/imu',Imu, self.readImu)
    self.thruster_sub = rospy.Subscriber('/bluerov2/thruster_manager/input',Wrench,self.thruster_callback)
    #self.pose_vehicle_sub = rospy.Subscriber('/bluerov2/pose_gt',Odometry,self.read_pose_callback)

  # -------   LIFE CYCLE   -------

  def write_to_file(self,data, package, dir):
    data_path = get_pkg_dir(package) + dir
    with open(data_path, 'w') as file:
        yaml.dump(data, file)
  
  def loop(self):
    while not rospy.is_shutdown():

      self.data_vel_meas.append(self.row_vm)
      self.data_thruster.append(self.row_t)
      self.omega_meas.append(self.row_omega_meas)
      
      print(' dnfn ')
      print(self.row_omega_meas)
      print(len(self.data_vel_meas))
        

      if len(self.data_vel_meas) == 21000:
        self.write_to_file(self.data_vel_meas,'bluerov2_control','/config/plot/vx_vz_meas.yaml')
        self.count = self.count +1
        print(3)

      if len(self.omega_meas) == 21000:
        self.write_to_file(self.omega_meas,'bluerov2_control','/config/plot/omega_meas.yaml')
        self.count = self.count +1
        print(4)

      if len(self.data_thruster) == 21000:
        self.write_to_file(self.data_thruster,'bluerov2_control','/config/plot/thruster_input.yaml')
        self.count = self.count +1
        print(5)  

      if self.count == 3:
        break

      self.rate.sleep()


  def readDVL(self,msg):

    v_meas_x = msg.velocity.x
    v_meas_z = msg.velocity.z
    self.row_vm = [v_meas_x,v_meas_z]
    


  def readImu(self,msg):
    omega = msg.angular_velocity
    self.row_omega_meas = omega.z
    


  def thruster_callback(self,msg):
    tx = msg.force.x
    tz = msg.force.z
    mz = msg.torque.z
    self.row_t = [tx,tz,mz]
    
  # -------   CALLBACK   -------
  #def read_callback(self,msg):
    #self.tz = rnd.choice([-3100, 3100])
    #self.input_pub.torque.z = self.tz
    #self.thruster_pub.publish(self.input_pub)
    #output_data = msg.angular_velocity.z
    #input_data = self.tz
    #self.row = [input_data,output_data]
    #self.data_errori.append(self.row)




# -------   INITIALIZE INSTANCE  -------
if __name__ == '__main__':
  node = plot_interno()
  node.loop()