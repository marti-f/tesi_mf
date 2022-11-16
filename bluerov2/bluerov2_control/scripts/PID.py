#!/usr/bin/env python3

import rospy
import numpy

class PID:
    def __init__(self,Kp,Ki=0,Kd=0,Tc=0.1,u_max=3000):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.Tc = Tc
        self.u_max = u_max
        self.Sk = 0
        self.ek_1 = 0

    def calculate(self,yk,r):
        ek = r-yk
        self.Sk += ek*self.Tc
        uk = self.Kp*ek +self.Ki*self.Sk +self.Kd*(ek-self.ek_1)/self.Tc
        self.ek_1 = ek
        if abs(uk) >= self.u_max:
            self.Sk -= ek*self.Tc
            return (uk/abs(uk))*self.u_max
        return uk

    def reset(self):
        self.Sk = 0
        seld.ek_1 = 0