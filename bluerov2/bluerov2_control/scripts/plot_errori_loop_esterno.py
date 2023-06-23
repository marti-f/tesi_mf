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

class plot_esterno:
  def __init__(self):
    rospy.init_node('plot_loop_esterno')

    # ------- PARAMETERS ------- 
    self.freq = 1/0.06
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
    self.row_omega_meas = []
    self.row_pose = []
    self.row_t = []
    self.row_vm = []
    self.row_vr = []
    


    # -------   TOPICS   -------    
    self.errori_plot_sub = rospy.Subscriber('/bluerov2/estimated_state', Pipeline_State,self.estimated_state_callback)
    #self.thruster_pub = rospy.Publisher('/bluerov2/thruster_manager/input',Wrench,queue_size=1)
    self.velo_riferimento = rospy.Subscriber('/bluerov2/cmd_vel',Twist,self.velo_callback)
    #self.vel_dvl_sub = rospy.Subscriber('/bluerov2/dvl',DVL,self.readDVL)
    #self.vel_imu_sub = rospy.Subscriber('/bluerov2/Imu',Imu, self.readImu)
    #self.thruster_sub = rospy.Subscriber('/bluerov2/thruster_manager/input',Wrench,self.thruster_callback)
    self.pose_vehicle_sub = rospy.Subscriber('/bluerov2/pose_gt',Odometry,self.read_pose_callback)

  # -------   LIFE CYCLE   -------

  def write_to_file(self,data, package, dir):
    data_path = get_pkg_dir(package) + dir
    with open(data_path, 'w') as file:
        yaml.dump(data, file)
  
  def loop(self):
    while not rospy.is_shutdown():
      print(' dnfn ')
      if len(self.data_errori) == 100:
        self.write_to_file(self.data_errori,'bluerov2_control','/config/plot/e_y_psi.yaml')
        self.count = self.count +1
        print(1)
        
      if len(self.data_vel_ref) == 100:
        self.write_to_file(self.data_vel_ref,'bluerov2_control','/config/plot/vel_riferimento.yaml')
        self.count = self.count +1
        print(2)

      if len(self.data_pose) == 100:
        self.write_to_file(self.data_pose,'bluerov2_control','/config/plot/posa_x_y.yaml')
        self.count = self.count +1
        print(6)
      if self.count == 3:
        break
      print(self.count)
      self.rate.sleep()

      

  def estimated_state_callback(self,msg):
    e_y = msg.y
    e_psi = msg.yaw
    self.row_e = [e_y,e_psi]
    self.data_errori.append(self.row_e)
  
  def velo_callback(self,msg):
    omega_ref = msg.angular.z
    vx_ref = msg.linear.x
    vz_ref = msg.linear.z
    self.row_vr = [vx_ref,vz_ref,omega_ref]
    self.data_vel_ref.append(self.row_vr)


  def read_pose_callback(self,msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    self.row_pose = [x,y]
    self.data_pose.append(self.row_pose)
    
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
  node = plot_esterno()
  node.loop()