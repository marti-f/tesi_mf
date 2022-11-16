#!/usr/bin/env python3

from bluerov2_control.msg import Line_Error 
from geometry_msgs.msg import Twist
import rospy
import math
import PID




class LineFollower:
    def __init__(self):
        rospy.init_node('line_follower')
        
    # ------- PARAMETERS ------- 
        self.freq = rospy.get_param('/gazebo/freq', 0.1)
        self.rate = rospy.Rate(self.freq) 
        
        self.pid_x = PID.PID(0.00025,0,0,1/self.freq,10)
        self.pid_theta = PID.PID(0.000413,0,0,1/self.freq,10)

    # ------- VARIABLES -------
        self.state = 'STOP'
        self.e_x = 0
        self.e_theta = 0 
        self.detect = 0 
    # -------   TOPICS   -------
        self.line_error_sub = rospy.Subscriber('/bluerov2/line_error',Line_Error,self.line_error_callback)
        self.vel_ref_pub = rospy.Publisher('/bluerov2/cmd_vel',Twist,queue_size=10)       


    # -------   LIFE CYCLE   -------
    def loop(self):
        rospy.spin()
  # -------   CALLBACK   -------
    def line_error_callback(self, msg):
        v_ref = Twist()
        self.e_x = msg.linear
        self.e_theta = msg.angular
        self.detect = msg.rect_detected
        self.nextState()
        if self.state == 'STOP':
            v_ref.linear.x = 0.0
            v_ref.linear.y = 0.0
            v_ref.linear.z = 0.0
            v_ref.angular.x = 0.0
            v_ref.angular.y = 0.0
            v_ref.angular.z = 0.0
        if self.state == 'CENTER':
            v_ref.linear.x = 0.09
            v_ref.linear.y = 0.0
            v_ref.linear.z = 0.0
            v_ref.angular.x = 0.0
            v_ref.angular.y = 0.0   
            v_ref.angular.z = (self.pid_x.calculate(self.e_x,0) + self.pid_theta.calculate(self.e_theta,0))
        if self.state == 'RIGHT':
            v_ref.linear.x = 0.02
            v_ref.linear.y = 0.0
            v_ref.linear.z = 0.0
            v_ref.angular.x = 0.0
            v_ref.angular.y = 0.0
            v_ref.angular.z = (self.pid_x.calculate(self.e_x,0) + self.pid_theta.calculate(self.e_theta,0))

        if self.state == 'LEFT':
            v_ref.linear.x = 0.02
            v_ref.linear.y = 0.0
            v_ref.linear.z = 0.0
            v_ref.angular.x = 0.0
            v_ref.angular.y = 0.0
            v_ref.angular.z = (self.pid_x.calculate(self.e_x,0) + self.pid_theta.calculate(self.e_theta,0))

        if self.state == 'DISALIGNED':
            v_ref.linear.x = 0.03
            v_ref.linear.y = 0.0
            v_ref.linear.z = 0.0
            v_ref.angular.x = 0.0
            v_ref.angular.y = 0.0
            v_ref.angular.z = 0.0
        print("STATE: ")
        print(self.state)

        if not(abs(v_ref.angular.z)<0.3):
            v_ref.angular.z =(v_ref.angular.z/abs(v_ref.angular.z))*0.3

        self.vel_ref_pub.publish(v_ref)


    def nextState(self):
        # Next State
        if self.detect:
            if abs(self.e_x) <= 10.5 and abs(self.e_theta) <= 22:
                self.state = 'CENTER'
            elif (self.e_x < -10.5 and abs(self.e_theta) <= 22) or (abs(self.e_x) <= 10.5 and self.e_theta < -22):
                self.state = 'LEFT'
            elif (self.e_x > 10.5 and abs(self.e_theta) <= 22) or (abs(self.e_x) <=10.5 and self.e_theta > 22):
                self.state = 'RIGHT'
            elif (self.e_x > 10.5 and self.e_theta < -22) or (self.e_x < -10.5 and self.e_theta > 22):
                self.state = 'DISALIGNED'
        else:
            self.state = 'STOP'


# -------   INITIALIZE INSTANCE  -------
if __name__ == '__main__':
    node = LineFollower()
    node.loop()