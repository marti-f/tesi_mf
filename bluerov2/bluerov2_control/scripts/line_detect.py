#!/usr/bin/env python3

from sensor_msgs.msg import Image 
import rospy
import numpy as np
import cv2, cv_bridge
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from bluerov2_control.msg import Line_Error
import math



class LineDetect:
    def __init__(self):
        rospy.init_node('line_detector')
    # ------- PARAMETERS ------- 
        self.freq = 1/0.06
        self.rate = rospy.Rate(self.freq)
        self.bridge = cv_bridge.CvBridge()
        self.blue_upper = (255,50,50)
        self.blue_lower = (150,0,0)
        self.angle_cam = 50
        self.count = 0

    # -------   TOPICS   -------
        self.image_sub = rospy.Subscriber('/bluerov2/camera_front/camera_image',Image,self.image_callback)
        self.error_detect_pub = rospy.Publisher('/bluerov2/line_error_camera',Line_Error,queue_size=1)
        # topic measure z_k = [x_k,y_k,phi]
        self.measure_camera_pub = rospy.Publisher('/bluerov2/measure_camera',Twist,queue_size = 1)

    # -------   LIFE CYCLE   -------
    def loop(self):
        while not rospy.is_shutdown():
             self.count = self.count +1
             self.rate.sleep()
  # -------   CALLBACK   -------
    def image_callback(self, msg):

        # Conversione dell'immagine ROV in formato OpenCV
        img_CV = self.bridge.imgmsg_to_cv2(msg,desired_encoding='passthrough')

        # Rimpicciolisco l'immagine del 50% delle sue dimensioni originali 
        dim_x = int(1920/3)
        dim_y = int(1080/3)
        dim =(dim_x,dim_y)
        img_BGR = cv2.resize(img_CV,dim,interpolation=cv2.INTER_AREA)
        img_RGB = cv2.cvtColor(img_BGR,cv2.COLOR_BGR2RGB)
        dim_y_new = int(dim_y/2)
        img_RGB =img_RGB[dim_y_new:dim_y,0:dim_x]
        #cv2.imshow('Immagine originale',img_RGB)
        #cv2.waitKey(1)
        dim_y = dim_y_new
        print(self.count)
        img_show = img_RGB.copy()

        cv2.imshow("image_not_blurred",img_show)
        cv2.waitKey(1)

        # Smussamento dell' immagine tramite filtro Gaussiano


        #############################################################################
        # DISTURBO SULL'IMMAGINE
        
        #if (800<=(self.count)<=1000)|(1700<=(self.count)<=1900):
            # Generate random Gaussian noise
        #    mean = 1000
        #    stddev = 10000
        #    noise = 150*np.ones(img_RGB.shape, np.uint8)
            #cv2.randn(noise, mean, stddev)
        #    img_RGB = cv2.subtract(img_RGB, noise)
        #    img_RGB = cv2.GaussianBlur(img_RGB,(51,51),10)
        #else:
        
        img_RGB = cv2.GaussianBlur(img_RGB,(5,5),5)
        img_show2 =img_RGB.copy()
        cv2.imshow("image_blurred",img_show2)
        cv2.waitKey(1)

        img_blurred = img_RGB.copy()
        img_RGB_copy = img_RGB.copy()
        
        # Canny Edge Detector
        #ratio = 3
        #kernel_size = 3
        #low_threshold = 80
        #src_gray = cv2.cvtColor(img_blurred,cv2.COLOR_RGB2GRAY)
        #detected_edge = cv2.Canny(src_gray,low_threshold,low_threshold*ratio,kernel_size)
        #mask_canny = detected_edge != 0
        #dst =img_RGB*(mask_canny[:,:,None].astype(img_RGB.dtype))
        


        # Hough Trasformation Image
        #detected_edge_gray = cv2.cvtColor(detected_edge,cv2.COLOR_GRAY2BGR)
        #lines = cv2.HoughLines(detected_edge,1,np.pi/180,360,0,0)

        #if lines is not None:
        #    x_avg = 0
        #    y_avg = 0
        #    for i in range(0,len(lines)):
        #        rho = lines[i][0][0]
        #        theta = lines[i][0][1]
        #        a = math.cos(theta)
        #        b = math.sin(theta)
        #        x0 = a * rho
        #        y0 = b * rho
        #        pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
        #        pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
        #        x_avg = x_avg + x0
        #        y_avg = y_avg + y0
        #        cv2.line(detected_edge_gray, pt1, pt2, (0,0,255), 1, cv2.LINE_AA)
        #    x_avg = x_avg/len(lines)
        #    y_avg = y_avg/len(lines)
        #    pt1_avg = (int(x_avg + 1000*(-b)), int(y_avg + 1000*(a)))
        #    pt2_avg = (int(x_avg - 1000*(-b)), int(y_avg - 1000*(a)))
        #    cv2.line(detected_edge_gray, pt1_avg, pt2_avg, (0,255,0), 3, cv2.LINE_AA)




        # Creazione della maschera per il tubo blu
        blue_mask = cv2.inRange(img_RGB_copy,self.blue_lower,self.blue_upper)   
        only_blue = cv2.bitwise_and(img_RGB_copy,img_RGB_copy, mask = blue_mask)
        
        cv2.imshow("image_mask",blue_mask)
        cv2.waitKey(1)
        
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

            x_close = x_cent -h_cent*math.sin(math.radians(ang))/2
            y_close = y_cent +h_cent*math.cos(math.radians(ang))/2

            # trasformazione di coordinate dal frame immagine a quello camera

            x_next_c,y_next_c,z_next_c = coordinate_image2camera(x_cent,y_cent,self)
            x_close_c,y_close_c,z_close_c = coordinate_image2camera(x_close,y_close,self)
            print('x_cent,y_cent= '+str(x_cent)+','+str(y_cent))
            print('x_next,y_next,z_next camera= '+str(x_next_c)+','+str(y_next_c)+','+str(z_next_c))
            print('x_close,y_close,z_close camera = '+str(x_close_c)+','+str(y_close_c)+','+str(z_close_c))

            # trasformazione di coordinate dal frame camera al frame solidale al veicolo
            x_next_v,y_next_v,z_next_v = camera2vehicle(x_next_c,y_next_c,z_next_c,0.22,0,0,self)

            x_close_v,y_close_v,z_close_v = camera2vehicle(x_close_c,y_close_c,z_close_c,0.22,0,0,self)

            e_orientamento = math.atan2(-y_next_v,x_next_v)
            e_orientamento = (e_orientamento*180/np.pi)

            print('Errore orizzontale = '+ str(y_close_v))
            print('Errore di orientamento = '+ str(e_orientamento))
            print('-------')

            
            # Calcolo dell' errore lineare
            setpoint = int(dim_x/2)*0.5
            error = x_cent-setpoint 

            # Inserimento del box pipe
            box = cv2.boxPoints(pipe_box)
            box = np.int0(box)
            cv2.drawContours(img_RGB,[box],0,(255,0,255),3)
            
            text = "e_l="+ str(np.ceil(y_close_v))
            text_ang = "e_a=" +str(np.ceil(e_orientamento))

            #cv2.putText(img_RGB,text_ang,(48,56),cv2.FONT_HERSHEY_SIMPLEX,1.5,(0,0,0),2)
            #cv2.putText(img_RGB,text,(48,300),cv2.FONT_HERSHEY_SIMPLEX,1.5,(0,0,0),2)

            # Publicazione dell'errore sul topic line_error
            error_pub = Line_Error()
            error_pub.linear = error
            error_pub.angular = ang
            error_pub.rect_detected = True
            self.error_detect_pub.publish(error_pub)

            # Pubblicazione del topic measure
            measure_pub = Twist()
            measure_pub.linear.x = x_close_v
            measure_pub.linear.y = y_close_v
            measure_pub.linear.z = z_close_v
            measure_pub.angular.z = e_orientamento
            self.measure_camera_pub.publish(measure_pub)


        else:
            error_pub = Line_Error()
            error_pub.linear = -1
            error_pub.angular = -1
            error_pub.rect_detected = False
            print("Nessun contorno rilevato")
            self.error_detect_pub.publish(error_pub)
        dim = (int(dim_x),int(dim_y))
        img_show = cv2.resize(img_RGB,dim,interpolation=cv2.INTER_AREA)
        cv2.imshow("image",img_show)
        cv2.waitKey(1)


def coordinate_image2camera(x,y,self):
    f = 0.0036 #focal length
    sin_th =math.sin(math.radians(self.angle_cam)) #sin(50)
    cos_th = math.sin(math.radians(self.angle_cam)) #cos(50)
    H = 1080/3 #pixel
    W = 1920/3 #pixel
    rho = 0.0000014
    c_x = W/2
    c_y = H/2
    h = 0.8 # altezza del veicolo dal fondale (metri)
    # conversione di coordinate dall' immagine al frame della camera
    zc = h/(sin_th + (((y)-(H/2))*rho)*cos_th/f)

    xc = ((x-(W/2))*zc*rho)/(f*sin_th)

    yc = (((y-(H/2))*zc*rho))/(f*sin_th)

    return xc,yc,zc

def camera2vehicle(xc,yc,zc,x0,y0,z0,self):
    #XC = np.array([[xc],[yc],[zc]])
    #X0 = np.array([[x0],[y0],[z0]])

    #R_theta = np.array([[1,0,0],[0,0.76,0.64],[0,-0.64,0.76]])
    #R_frame = np.array([[0,0,1],[1,0,0],[0,,0]])

    #M = np.dot(R_frame,R_theta)

    #XV = X0 + np.dot(M,XC)
    sin_th =math.sin(math.radians(self.angle_cam)) #sin(50)
    cos_th = math.sin(math.radians(self.angle_cam)) #cos(50)

    xv = x0 -yc*sin_th+zc*cos_th
    yv = y0 - xc
    zv = z0 -yc*cos_th-zc*sin_th

    return xv,yv,zv

# -------   INITIALIZE INSTANCE  -------
if __name__ == '__main__':
    node = LineDetect()
    node.loop()