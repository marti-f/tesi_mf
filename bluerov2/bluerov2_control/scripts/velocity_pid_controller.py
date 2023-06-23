#!/usr/bin/env python3
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
from uuv_sensor_ros_plugins_msgs.msg import DVL
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import Vector3,Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


import PID
import rospy




class VelocityPIDController:


    def __init__(self):
        rospy.init_node('velo_PID_controller')
    # ------- PARAMETERS ------- 
        self.Tc = 0.02
        self.freq = 1/self.Tc
        self.rate = rospy.Rate(self.freq)
        
        # Saturazione dei thruster
        
    # ------- VARIABLES -------

        # PID Controller
        self.pid_surge = PID.PID(494.5529,954.26,0,self.Tc,3100)
        self.pid_heave = PID.PID(334.65,3366.766,0,self.Tc,3100)
        self.pid_yaw = PID.PID(1333,0,0,self.Tc,1040)

        # Velocita lineare letta
        self.v_meas = Vector3()

        # Velocita` angolare
        self.omega_meas = Vector3()

        # Velocita` di riferimento
        self.v_ref = Twist()

    # -------   TOPICS   -------
        #rospy.Subscriber('/bluerov2/pose_body',Odometry,self.read_pose_callback)
        rospy.Subscriber('/bluerov2/dvl',DVL,self.readDVL)
        rospy.Subscriber('/bluerov2/imu',Imu, self.readImu)
        self.writer_thruster_pub = rospy.Publisher('/bluerov2/thruster_manager/input',Wrench,queue_size=1)
        rospy.Subscriber('/bluerov2/cmd_vel',Twist,self.cmd_vel_callback)

    # -------   LIFE CYCLE   -------
    def loop(self):
        while not rospy.is_shutdown():
        # node operation
            #  Calcolo dell' azione di controllo 
            input = Wrench()
            
            input.force.x = self.pid_surge.calculate(self.v_meas.x,self.v_ref.linear.x)
            input.force.z = self.pid_heave.calculate(self.v_meas.z,self.v_ref.linear.z)
            input.torque.z = self.pid_yaw.calculate(self.omega_meas.z,self.v_ref.angular.z)

            # Pubblicazione del messaggio sul topic
            self.writer_thruster_pub.publish(input)
            self.rate.sleep()

  # -------   CALLBACK   -------
    def readDVL(self,msg):
        self.v_meas = msg.velocity

    def readImu(self,msg):
        self.omega_meas = msg.angular_velocity
      

    def cmd_vel_callback(self,msg):
        self.v_ref = msg

# -------   INITIALIZE INSTANCE  -------
if __name__ == '__main__':
    node = VelocityPIDController()
    node.loop()