#!/usr/bin/env python3
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import Vector3,Twist
from nav_msgs.msg import Odometry

import PID
import rospy




class VelocityPIDController:


    def __init__(self):
        rospy.init_node('velo_PID_controller')
    # ------- PARAMETERS ------- 
        self.freq = rospy.get_param('/gazebo/freq', 100)
        self.rate = rospy.Rate(self.freq)
        self.Tc = 0.01
        # Saturazione dei thruster
        
    # ------- VARIABLES -------

        # PID Controller
        self.pid_surge = PID.PID(2033,653,0,self.Tc,3100)
        self.pid_heave = PID.PID(145000,1100,3110,self.Tc,3000)
        self.pid_yaw = PID.PID(930,583,0,self.Tc,1040)

        # Velocita letta
        self.v_meas = Twist()

        # Velocita` di riferimento
        self.v_ref = Twist()

    # -------   TOPICS   -------
        rospy.Subscriber('bluerov2/pose_body',Odometry,self.read_pose_callback)
        self.writer_thruster_pub = rospy.Publisher('/bluerov2/thruster_manager/input',Wrench,queue_size=1)
        rospy.Subscriber('/bluerov2/cmd_vel',Twist,self.cmd_vel_callback)

    # -------   LIFE CYCLE   -------
    def loop(self):
        while not rospy.is_shutdown():
        # node operation
            #  Calcolo dell' azione di controllo 
            input = Wrench()
            
            input.force.x = self.pid_surge.calculate(self.v_meas.linear.x,self.v_ref.linear.x)
            input.force.z = self.pid_heave.calculate(self.v_meas.linear.z,self.v_ref.linear.z)
            input.torque.z = self.pid_yaw.calculate(self.v_meas.angular.z,self.v_ref.angular.z)

            # Pubblicazione del messaggio sul topic
            self.writer_thruster_pub.publish(input)
            self.rate.sleep()

  # -------   CALLBACK   -------
    def read_pose_callback(self, msg):
        
        # operations on msg coming from topic'
        self.v_meas = msg.twist.twist
      

    def cmd_vel_callback(self,msg):
        self.v_ref = msg

# -------   INITIALIZE INSTANCE  -------
if __name__ == '__main__':
    node = VelocityPIDController()
    node.loop()