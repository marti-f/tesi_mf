#!/usr/bin/env python3

from sensor_msgs.msg import Image 
import rospy
import numpy as np
import cv2, cv_bridge
from std_msgs.msg import Bool
from bluerov2_control.msg import Line_Error
import math



class LineDetect:
    def __init__(self):
        rospy.init_node('line_detector')
    # ------- PARAMETERS ------- 
        self.freq = 1/rospy.get_param('/gazebo/time_step', 0.1)
        self.rate = rospy.Rate(self.freq)
        self.bridge = cv_bridge.CvBridge()
        self.blue_upper = (255,50,50)
        self.blue_lower = (0,0,0)

    # -------   TOPICS   -------
        self.image_sub = rospy.Subscriber('/bluerov2/camera_front/camera_image',Image,self.image_callback)
        self.error_detect_pub = rospy.Publisher('/bluerov2/line_error',Line_Error,queue_size=1)

    # -------   LIFE CYCLE   -------
    def loop(self):
        rospy.spin()
  # -------   CALLBACK   -------
    def image_callback(self, msg):

        # Conversione dell'immagine ROV in formato OpenCV
        img_CV = self.bridge.imgmsg_to_cv2(msg,desired_encoding='passthrough')

        # Rimpicciolisco l'immagine del 60% delle sue dimensioni originali 
        dim_x = int(1920*0.5)
        dim_y = int(1080*0.5)
        dim =(dim_x,dim_y)
        img_BGR = cv2.resize(img_CV,dim,interpolation=cv2.INTER_AREA)
        img_RGB = cv2.cvtColor(img_BGR,cv2.COLOR_BGR2RGB)
        img_RGB =img_RGB[180:dim_y,0:dim_x]
        img_RGB = cv2.GaussianBlur(img_RGB,(5,5),0)
        img_RGB_copy = img_RGB.copy()

        # Creazione della maschera per il tubo blu
        blue_mask = cv2.inRange(img_RGB_copy,self.blue_lower,self.blue_upper)   
        only_blue = cv2.bitwise_and(img_RGB_copy,img_RGB_copy, mask = blue_mask)

        # Calcolo dei contorni dell' immagine 
        contour,hierarchy = cv2.findContours(blue_mask.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

        if len(contour) > 0:
            if len(contour) == 1:
                # Calcolo del box minimo (ruotato)
                pipe_box = cv2.minAreaRect(contour[0])
            else:
                ## Eliminazione dei falsi positivi
                corners = []
                for cont in range(len(contour)):
                    # Calcolo del box minimo (ruotato) del primo contorno ricavato
                    pipe_box_cont = cv2.minAreaRect(contour[cont])
                    # Calcolo dei vertici del rettangolo ricavato
                    pipe_box_cont = cv2.boxPoints(pipe_box_cont)
                    # Selezione del box con il vertice piu' basso
                    (x_box,y_box) = pipe_box_cont[0]
                    corners.append((y_box,cont))
                    # Ordinamento decrescente
                    corners = sorted(corners,reverse=True)
                (y_high,cont_high) = corners[0]
                pipe_box = cv2.minAreaRect(contour[cont_high])
                            
            # Inserimento di tutti i contorni nell'immagine (-1)
            cv2.drawContours(img_RGB, contour, -1, (0,255,0), 3)
            (x_cent,y_cent),(w_cent,h_cent),ang = pipe_box
           
            # Conversione dell'angolo minAreaRect
            if ang >= 45:
                ang = -90+ang
            if w_cent < h_cent and ang < 0:
                    ang = 90+ang
            if w_cent > h_cent and ang > 0:
                    ang = (-90+ang)
            
            # Calcolo dell' errore lineare
            setpoint = 960*0.5
            error = x_cent-setpoint 

            # Inserimento del box pipe
            box = cv2.boxPoints(pipe_box)
            box = np.int0(box)
            cv2.drawContours(img_RGB,[box],0,(255,0,255),3)
            
            text = "e_l="+ str(error)
            text_ang = "e_a=" +str(ang)

            cv2.putText(img_RGB,text_ang,(48,56),cv2.FONT_HERSHEY_SIMPLEX,1.5,(0,0,0),2)
            cv2.putText(img_RGB,text,(48,300),cv2.FONT_HERSHEY_SIMPLEX,1.5,(0,0,0),2)

            # Publicazione dell'errore sul topic d' interesse
            error_pub = Line_Error()
            error_pub.linear = error
            error_pub.angular = ang
            error_pub.rect_detected = True
            self.error_detect_pub.publish(error_pub)
        else:
            error_pub = Line_Error()
            error_pub.linear = -1
            error_pub.angular = -1
            error_pub.rect_detected = False
            print("Nessun contorno rilevato")
            self.error_detect_pub.publish(error_pub)

        cv2.imshow("image",img_RGB)
        cv2.waitKey(1)

# -------   INITIALIZE INSTANCE  -------
if __name__ == '__main__':
    node = LineDetect()
    node.loop()