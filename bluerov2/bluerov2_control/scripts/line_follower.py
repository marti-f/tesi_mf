#!/usr/bin/env python3

from bluerov2_control.msg import Pipeline_State 
from geometry_msgs.msg import Twist
import rospy
import math
import PID




class LineFollower:
    def __init__(self):
        rospy.init_node('line_follower')
        
    # ------- PARAMETERS ------- 
        self.T = 0.06
        self.freq = 1/self.T
        self.rate = rospy.Rate(self.freq) 
        
        self.pid_y = PID.PID(-0.1,0,0,1/self.freq,10)
        self.pid_theta = PID.PID(0.014,0,0,1/self.freq,10)

    # ------- VARIABLES -------
        self.state = 'STOP'
        self.e_horizontal = 0
        self.e_orientation = 0 

        self.v_ref = Twist()
        
    # -------   TOPICS   -------
        self.estimated_state_sub = rospy.Subscriber('/bluerov2/estimated_state',Pipeline_State,self.estimated_state_callback)
        self.vel_ref_pub = rospy.Publisher('/bluerov2/cmd_vel',Twist,queue_size=10)       


    # -------   LIFE CYCLE   -------
    def loop(self):
        while not rospy.is_shutdown():
             self.vel_ref_pub.publish(self.v_ref)
             self.rate.sleep()

  # -------   CALLBACK   -------
    def estimated_state_callback(self, msg):
        
        self.e_horizontal = msg.y
        self.e_orientation = msg.yaw
        self.nextState()
        if self.state == 'STOP':
            self.v_ref.linear.x = 0.0
            self.v_ref.linear.y = 0.0
            self.v_ref.linear.z = 0.0
            self.v_ref.angular.x = 0.0
            self.v_ref.angular.y = 0.0
            self.v_ref.angular.z = 0.0
        if self.state == 'CENTER':
            self.v_ref.linear.x = 0.07
            self.v_ref.linear.y = 0.0
            self.v_ref.linear.z = 0.0
            self.v_ref.angular.x = 0.0
            self.v_ref.angular.y = 0.0   
            self.v_ref.angular.z = (self.pid_y.calculate(self.e_horizontal,0) + self.pid_theta.calculate(self.e_orientation,0))
        if self.state == 'RIGHT':
            self.v_ref.linear.x = 0.02
            self.v_ref.linear.y = 0.0
            self.v_ref.linear.z = 0.0
            self.v_ref.angular.x = 0.0
            self.v_ref.angular.y = 0.0
            self.v_ref.angular.z = (self.pid_y.calculate(self.e_horizontal,0) + self.pid_theta.calculate(self.e_orientation,0))

        if self.state == 'LEFT':
            self.v_ref.linear.x = 0.02
            self.v_ref.linear.y = 0.0
            self.v_ref.linear.z = 0.0
            self.v_ref.angular.x = 0.0
            self.v_ref.angular.y = 0.0
            self.v_ref.angular.z = (self.pid_y.calculate(self.e_horizontal,0) + self.pid_theta.calculate(self.e_orientation,0))

        if self.state == 'DISALIGNED':
            self.v_ref.linear.x = 0.05
            self.v_ref.linear.y = 0.0
            self.v_ref.linear.z = 0.0
            self.v_ref.angular.x = 0.0
            self.v_ref.angular.y = 0.0
            self.v_ref.angular.z = 0.0

        
        print("STATE: "+ self.state)
        print('cmd_vel')
        print('vx = '+str(self.v_ref.linear.x))
        print('omega_z = '+str(self.v_ref.angular.z))
        print('ERRORI')
        print('e_horizontal = '+str(self.e_horizontal))
        print('e_orientation = '+str(self.e_orientation))

        if not(abs(self.v_ref.angular.z)<0.3):
            self.v_ref.angular.z =(self.v_ref.angular.z/abs(self.v_ref.angular.z))*0.3

       


    def nextState(self):
        # Next State
        if abs(self.e_horizontal) <= 0.02 and abs(self.e_orientation) <= 2:
            self.state = 'CENTER'
        elif (self.e_horizontal < -0.02 and (self.e_orientation) >= 2) or (abs(self.e_horizontal) <= 0.02 and self.e_orientation > -2):
            self.state = 'LEFT'
        elif (self.e_horizontal > 0.02 and (self.e_orientation) <= 2) or (abs(self.e_horizontal) <=0.02 and self.e_orientation < 2):
            self.state = 'RIGHT'
        elif (self.e_horizontal < -0.02 and self.e_orientation < 2) or (self.e_horizontal > 0.02 and self.e_orientation > -2):
            self.state = 'DISALIGNED'
        else:
            self.state = 'STOP'


# -------   INITIALIZE INSTANCE  -------
if __name__ == '__main__':
    node = LineFollower()
    node.loop()