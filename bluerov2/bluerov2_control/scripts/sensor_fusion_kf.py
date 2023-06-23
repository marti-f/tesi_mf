#!/usr/bin/env python3
import rospy
import numpy as np
from bluerov2_control.msg import Pipeline_State
from geometry_msgs.msg import Twist


class SensorFusion:

    def __init__(self):
        rospy.init_node('sensor_fusion_node')



        # -------   VARIABLES   -------
        self.T = 0.06
        self.freq = 1/self.T 
        self.rate = rospy.Rate(self.freq)
        
        # Parametri del sistema

        # x_(k+1) = F*x_k + W_k
        # z_k = H*x_k +V_k

        # matrice di transizione dello stato
        self.F = np.array([[1,self.T, 0, 0, 0, 0],[0,1, 0, 0, 0, 0],[0, 0, 1, self.T, 0, 0],[0, 0, 0, 1, 0, 0],[0, 0, 0, 0, 1, self.T],[0, 0, 0, 0, 0, 1]])       # matrice di uscita
        self.H = np.array([[1, 0, 0, 0, 0, 0],[0, 0, 1, 0,0, 0],[0, 0, 0, 0, 1,0],[0, 0, 0, 0, 1, 0]])

        # Parametri del filtro 
        # matrice di covarianza del rumore di processo Q
        self.Q = np.array([[0.01,0,0,0,0, 0],[0, 0.25, 0, 0, 0, 0],[0, 0, 0.01, 0, 0, 0],[0, 0, 0, 0.25, 0, 0],[0, 0, 0, 0, 0.01, 0],[0, 0, 0, 0, 0, 0.25]])

        # matrice di covarianza del rumore di misura R
        self.R = np.array([[0.0001, 0, 0, 0],[0, 0.0001, 0, 0],[0, 0, 0.0001, 0],[0, 0, 0, 0.01]])

        # inizializzazione del filtro
        self.state = np.transpose(np.array([[0, 0.1, 0, 0.1, 0, 0.1]]))

        self.P = np.eye(6)

        # variabili di misura:
        self.z_meas = np.zeros(4)

        self.meas_camera = Twist()
        self.meas_sonar = Twist()
        self.K = np.zeros((6,6))


        # -------   TOPICS   -------
        # subscriber
        self.meas_camera_sub = rospy.Subscriber('/bluerov2/measure_camera', Twist,self.meas_camera_callback)
        self.meas_sonar_pub = rospy.Subscriber('/bluerov2/measure_sonar',Twist,self.meas_sonar_callback)
        # publisher
        self.estimated_state = rospy.Publisher('/bluerov2/estimated_state', Pipeline_State, queue_size=1)




    # -------   LIFE CYCLE   -------
    def loop(self):
        while not rospy.is_shutdown():

            #inserisco le misure nel vettore z_meas = [z_cam; z_son]
            self.z_meas = np.array([[self.meas_camera.linear.x],[self.meas_camera.linear.y],[self.meas_camera.angular.z],[self.meas_sonar.angular.z]])

            # GUADAGNO DI KALMAN K
            # K = F*P*H'(H*P*H' + R)^-1
            N = np.linalg.inv(np.dot(self.H,np.dot(self.P,np.transpose(self.H))) + self.R)
            self.K = np.dot(self.F,np.dot(self.P,np.dot(np.transpose(self.H),N)))

            # STIMA DELLO STATO X_k
            # innovazione
            E = self.z_meas - np.dot(self.H,self.state)

            #stato stimato 
            self.state = np.dot(self.F,self.state) + np.dot(self.K,E)

            #aggiornamento della matrice P
            
            self.P = np.dot(self.F,np.dot(self.P,np.transpose(self.F)))+ self.Q -np.dot(self.K,np.dot(N,np.transpose(self.K)))
            print(self.state)
            print('-----------------------------')

            # pubblico lo stato sul topic
            state = Pipeline_State()
            state.x = self.state[0]
            state.vx = self.state[1]
            state.y = self.state[2]
            state.vy = self.state[3]
            state.yaw = self.state[4]
            state.omega_z = self.state[5]
            self.estimated_state.publish(state)
            self.rate.sleep()
    
    # -------   CALLBACK  -------
    def meas_camera_callback(self,msg):
        self.meas_camera = msg 

    def meas_sonar_callback(self,msg):
        self.meas_sonar = msg 

# -------   INITIALIZE INSTANCE  -------
if __name__ == '__main__':
    node = SensorFusion()
    node.loop()