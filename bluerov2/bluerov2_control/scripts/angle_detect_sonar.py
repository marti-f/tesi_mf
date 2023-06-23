#!/usr/bin/env python3

import rospy
import cv2, cv_bridge
from sensor_msgs.msg import Image 
import math
import numpy as np
import pywt
from statistics import mean
from bluerov2_control.msg import Line_Error
from geometry_msgs.msg import Twist

class sonar_detect:
    def __init__(self):
        rospy.init_node('angle_sonar_detect')
    # ------- PARAMETERS ------- 
        self.freq = 10
        self.rate = rospy.Rate(self.freq)
        self.bridge = cv_bridge.CvBridge()
        self.counter = 0
        self.counter_1= 0
        self.angle_RT = 0

        # Parametri del sonar
        self.fov = 40
        self.range_min = 0.75
        self.range_max = 1.6
        self.blue_lower = (0,0,0)
        self.blue_upper = (255,10,20)

        
    # -------   TOPICS   -------
        self.sonar_image_sub = rospy.Subscriber('/bluerov2/MSISonar_ping360',Image,self.sonar_image_callback)
        self.error_detect_pub = rospy.Publisher('/bluerov2/line_error_sonar',Line_Error,queue_size=1)
        # measure topic z_k = [x,y,phi]
        self.measure_sonar_pub = rospy.Publisher('/bluerov2/measure_sonar',Twist,queue_size = 1)

    # -------   LIFE CYCLE   -------
    def loop(self):
        rospy.spin()

  # -------   CALLBACK   -------
    def sonar_image_callback(self, msg):
       
        sonar_img_CV = self.bridge.imgmsg_to_cv2(msg,desired_encoding='passthrough')

        # copia per la conversione in coordinate polari
        sonar_polar = sonar_img_CV.copy()

        dim_x,dim_y,channel = sonar_img_CV.shape
        origin =(int(dim_x/2),int(dim_y/2))

        # CONVERSIONE DA COORDINATE POLARI A CARTESIANE
        
        sonar_img_cartesian = np.zeros(shape = (720,720))
        # ruoto l' immagine sonar di 180 gradi (per facilitare la conversione)
        # sonar_polar = cv2.flip(sonar_polar, 0)
        
        # effettuo la conversione
        dim_w = 720
        dim_h = 720*2

        sonar_img_cartesian= cv2.warpPolar(sonar_polar[:,:],dsize=(dim_w,dim_h),center= origin,maxRadius=360,flags = cv2.WARP_POLAR_LINEAR)
        # ruoto l' immagine convertita e taglio solo la parte del sonar
        
        
        sonar_img_cartesian = cv2.rotate(sonar_img_cartesian,cv2.ROTATE_90_COUNTERCLOCKWISE)
        #sonar_img_cartesian = cv2.flip(sonar_img_cartesian,1)
        h,w,chl= sonar_img_cartesian.shape
        
        sonar_img_cartesian = sonar_img_cartesian[:,int(w/2):dim_h]
 
        # taglio l' immagine in modo tale da prelevare solo il cono di interesse 
        # dm = k_phi * angle
        angle_min = 5*np.pi/18 # 50 gradi
        angle_max = 5*np.pi/18+8/18*np.pi #50 +80 gradi
        K_phi = dim_h/(2*np.pi)
        dm = int(K_phi*angle_min)
        dM = int(K_phi*angle_max)

        # taglio l' immagine in modo tale da selezionare solo la parte in cui c'e` la pipeline
        # r = k_lin * magnitude
        maxRadius = 360
        R_px = 360
        RM = 1.6-0.75
        r_ground = 1.25 #h/sin(40)
        r_pipeline = 0.1
        print(r_ground-r_pipeline-0.01)
        rm = r_ground-0.01-0.75
        rM = r_ground+r_pipeline+0.02-0.75
        print(rm)
        magnitude_min = (R_px/RM)*rm
        magnitude_max = (R_px/RM)*rM
        K_lin = dim_w/maxRadius
        rm_px = dim_w-int(K_lin*magnitude_min)
        rM_px = dim_w-int(K_lin*magnitude_max)

        sonar_img_flipped = sonar_img_cartesian[rM_px:rm_px,dm:dM] 
        # Applico un filtro
        sonar_img_flipped = cv2.GaussianBlur(sonar_img_flipped,(5,5),14,14)
        # Applico una maschera che prelevi solo il colore dell' ombra (blu)
        blue_mask = cv2.inRange(sonar_img_flipped,self.blue_lower,self.blue_upper)
        only_blue = cv2.bitwise_and(sonar_img_flipped,sonar_img_flipped,mask = blue_mask)

        # Ricavo i contorni della pipeline
        contour_blue,hierarchy_blue = cv2.findContours(blue_mask.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        # Calcolo dei rettangoli contenenti i contorni della pipeline
        #   > Eliminazione dei falsi positivi ( prelevo il rettangolo con il primo vertice piu` in basso)
        #   > eliminazione dei rettangoli con area minore di 50
        cv2.imshow('maschera ',blue_mask)
        cv2.waitKey(1)
        if len(contour_blue) > 0:
            corners_blue = []
            if len(contour_blue) == 1:
                # Calcolo del box minimo (ruotato)
                pipe_box_blue = cv2.minAreaRect(contour_blue[0])
            else:
                ## Eliminazione dei falsi positivi
                for cont in range(len(contour_blue)):
                    # Calcolo del box minimo (ruotato) del primo contorno ricavato
                    pipe_box_cont_blue = cv2.minAreaRect(contour_blue[cont])
                    area_b = pipe_box_cont_blue[1][0]*pipe_box_cont_blue[1][1]
            
                    if 5000 <= area_b <= 12000:
                    # Calcolo dei vertici del rettangolo ricavato
                        pipe_box_cont_blue = cv2.boxPoints(pipe_box_cont_blue)
                        # Selezione del box con il vertice piu' basso
                        (x_box_b,y_box_b) = pipe_box_cont_blue[0]
                        corners_blue.append((y_box_b,cont))
                        # Ordinamento decrescente
                        corners_blue = sorted(corners_blue,reverse=True)
                        print(area_b)
                
                print('Numero rettangoli = '+ str(len(corners_blue)))
                print('---')

                if len(corners_blue)>0:
                    (y_high_b,cont_high_b) = corners_blue[0]
                    pipe_box_blue = cv2.minAreaRect(contour_blue[cont_high_b])
                            
            # Inserimento di tutti i contorni nell'immagine (-1)
            if len(corners_blue)>0:
                cv2.drawContours(sonar_img_flipped, contour_blue, -1, (0,0,0), 3)
                (x_cent_b,y_cent_b),(w_cent_b,h_cent_b),ang_b = pipe_box_blue
                
                x_originale = -(y_cent_b+rM_px)
                y_originale = x_cent_b+dm+dim_h/2

                phi = (y_originale/K_phi)*180/np.pi
                phi = 360 -phi
                phi = phi-80
                #phi = (((x_cent_b+dm)/K_phi)*180/np.pi-90)
                ang_b_cartesian = ang_b
                
                # calcolo del raggio(distanza dell' oggetto dal sonar)
                dist_obj_px = (y_cent_b-dim_w)/K_lin
                dist_obj_meters = (RM/R_px)*dist_obj_px # conversione da pixel in metri

                # calcolo delle coordinate x,y,z dell' oggetto nel frame di coordinate del sonar
                h_sonar = 0.8
                x_sonar = dist_obj_meters*math.sin(math.radians(phi))
                y_sonar = dist_obj_meters*math.cos(math.radians(phi))
                z_sonar = h_sonar/math.cos(math.radians(40))

                # conversione delle coordinate dal frame sonar al frame del veicolo

                x_vehicle,y_vehicle,z_vehicle = sonar2vehicle(x_sonar,y_sonar,z_sonar,0.22,0,0,40)

                print('phi = '+ str(phi))
                print('x_v,y_v,z_v'+str(x_vehicle)+str(y_vehicle)+str(z_vehicle))

                if ang_b >= 45:
                    ang_b = -90+ang_b
                if w_cent_b < h_cent_b and ang_b < 0:
                    ang_b = 90+ang_b
                if w_cent_b> h_cent_b and ang_b > 0:
                    ang_b = (-90+ang_b)
                
                # Pubblicazione dell'errore sul topic d' interesse
                h_b,w_b,ch_b = sonar_img_flipped.shape
                setpoint = w_b/2
                error = x_cent_b-setpoint
                error = error

                text_ang = "e_a=" +str(np.ceil(ang_b))
                text_err = "e_l=" +str(np.ceil(error))


                error_pub = Line_Error()
                error_pub.linear = error
                error_pub.angular = ang_b
                error_pub.rect_detected = True
                self.error_detect_pub.publish(error_pub)               

                # Publicazione measure_topic 
                measure_pub = Twist()
                measure_pub.linear.x = x_vehicle
                measure_pub.linear.y = y_vehicle
                measure_pub.angular.z = phi

                self.measure_sonar_pub.publish(measure_pub)
                # Inserimento box pipe
                x_cart =x_cent_b+dm
                y_cart = y_cent_b+rM_px
                pipe_box_blue_cartesian=(x_cart,y_cart),(w_cent_b,h_cent_b),ang_b_cartesian
                
                box_b_cartesian = cv2.boxPoints(pipe_box_blue_cartesian)
                box_b_cartesian = np.int0(box_b_cartesian)
                box_b = cv2.boxPoints(pipe_box_blue)
                box_b = np.int0(box_b)
                
                
                cv2.drawContours(sonar_img_flipped,[box_b],0,(255,0,255),3)
                cv2.drawContours(sonar_img_cartesian,[box_b_cartesian],0,(255,0,255),3)
                cv2.imshow('immagine con box',sonar_img_cartesian)
                cv2.waitKey(1)
                
            else: 
                print('Nessun contorno rilevato')
                print(area_b)


                error_pub = Line_Error()
                error_pub.linear = -1
                error_pub.angular = -1
                error_pub.rect_detected = False
                self.error_detect_pub.publish(error_pub)                  

            




        #########################################################################
        #Inserimento dei bordi, linee e separatori
        white = (255,255,255)
        axes = origin
        axes_2 = (int(dim_x/8),int(dim_y/8))
        axes_3 = (int(dim_x/4),int(dim_y/4))
        axes_4 = (int(3*dim_x/8),int(3*dim_y/8))
        alpha = 3
        sonar_img_CV = cv2.ellipse(sonar_img_CV,origin,axes,-90,-self.fov-alpha,self.fov+alpha,white,1)
        sonar_img_CV = cv2.ellipse(sonar_img_CV,origin,axes_2,-90,-self.fov-alpha,self.fov+alpha,white,1)
        sonar_img_CV = cv2.ellipse(sonar_img_CV,origin,axes_3,-90,-self.fov-alpha,self.fov+alpha,white,1)
        sonar_img_CV = cv2.ellipse(sonar_img_CV,origin,axes_4,-90,-self.fov-alpha,self.fov+alpha,white,1)

        left_corner = (int(dim_y/2)+int((dim_x/2)*math.sin(np.pi/180.*(-self.fov-alpha))),int(dim_x/2)-int((dim_y/2)*math.cos(np.pi/180.*(-self.fov-alpha))))
        right_corner = (int(dim_y/2)+int((dim_x/2)*math.sin(np.pi/180.*(self.fov+alpha))),int(dim_x/2)-int((dim_y/2)*math.cos(np.pi/180.*(self.fov+alpha))))
        cv2.line(sonar_img_CV, origin, left_corner, white,1)
        cv2.line(sonar_img_CV, origin, right_corner, white,1)
        

        for i in range (0,3):
            angle = -self.fov/2.-0.5 + (self.fov+0.5)*i/(3-1)
            cornerx = int((dim_x/2)*math.sin(np.pi/180.*angle))
            cornery = int((dim_x/2)*math.cos(np.pi/180.*angle))
            corner = (int(dim_y/2)+cornerx, int(dim_x/2)-cornery)
            cv2.line(sonar_img_CV, origin, corner, white,1)
        
        # Line in real-time
        
        #if self.angle_RT <= -self.fov:
        #   self.angle_RT += 10
        #   self.counter_1 = self.counter
        #   self.counter = self.counter + 1    
        #elif self.angle_RT >= self.fov:
        #    self.angle_RT -= 10
        #    self.counter_1 = self.counter
        #    self.counter -= 1
        #elif self.angle_RT == 0:
        #    if self.counter-self.counter_1 > 0:
        #        self.angle_RT += 10
        #        self.counter_1 = self.counter
        #        self.counter = self.counter + 1
        #    else:
        #        self.angle_RT -= (10)
        #        self.counter_1 = self.counter
        #        self.counter = self.counter - 1     
        #elif -self.fov < self.angle_RT < 0:
        #    if self.counter-self.counter_1 >= 0:
        #        self.angle_RT += 10
        #        self.counter_1 = self.counter
        #        self.counter = self.counter + 1
        #    else:
        #        self.angle_RT -= 10
        #        self.counter_1 = self.counter
        #        self.counter -= 1 
        #elif 0 < self.angle_RT < self.fov:
        #    if self.counter-self.counter_1 <= 0:
        #        self.angle_RT -= (10)
        #        self.counter_1 = self.counter
        #        self.counter = self.counter - 1
        #    else:
        #        self.angle_RT += 10
        #        self.counter_1 = self.counter
        #        self.counter = self.counter + 1

           

        
        cornerx_RT= int((dim_x/2)*math.sin(np.pi/180.*self.angle_RT))
        cornery_RT = int((dim_x/2)*math.cos(np.pi/180.*self.angle_RT))
        corner_RT = (int(dim_y/2)+cornerx_RT, int(dim_x/2)-cornery_RT)
        #cv2.line(sonar_img_CV, origin, corner_RT, white,2)
        R = self.range_max-self.range_min
        R = R/4

        text = str(np.round(self.range_max,3)) + "meters"

        text_2 = str(np.round(self.range_max - 3*R,3))+ "meters"
        corner_2 = (int(dim_y/2)+int(((dim_x/2)/4)*math.sin(np.pi/180.*(self.fov+alpha))),1+int(dim_x/2)-int(((dim_y/2)/4)*math.cos(np.pi/180.*(self.fov+alpha))))
        
        text_3 = str(np.round(self.range_max - 2*R,3)) + "meters"
        corner_3 = (int(dim_y/2)+int(((dim_x/2)/2)*math.sin(np.pi/180.*(self.fov+alpha))),1+int(dim_x/2)-int(((dim_y/2)/2)*math.cos(np.pi/180.*(self.fov+alpha))))
        
        text_4 = str(np.round(self.range_max - R,3)) + "meters"
        corner_4 = (int(dim_y/2)+int((3*(dim_x/2)/4)*math.sin(np.pi/180.*(self.fov+alpha))),1+int(dim_x/2)-int((3*(dim_y/2)/4)*math.cos(np.pi/180.*(self.fov+alpha))))
        
        text_5 = str(np.round(self.range_min,3)) + 'meters'
        corner_5 =  origin

        cv2.putText(sonar_img_CV,text,right_corner,cv2.FONT_HERSHEY_SIMPLEX,0.5,white,2)
        cv2.putText(sonar_img_CV,text_2,corner_2,cv2.FONT_HERSHEY_SIMPLEX,0.5,white,2)
        cv2.putText(sonar_img_CV,text_3,corner_3,cv2.FONT_HERSHEY_SIMPLEX,0.5,white,2)
        cv2.putText(sonar_img_CV,text_4,corner_4,cv2.FONT_HERSHEY_SIMPLEX,0.5,white,2)
        cv2.putText(sonar_img_CV,text_5,corner_5,cv2.FONT_HERSHEY_SIMPLEX,0.5,white,2)
        #########################################################################


        ## Seleziono le coordinate dell' area di interesse

        
        #Datatype conversions
        #convert to grayscale
        #img_gray = cv2.cvtColor( sonar_img_CV,cv2.COLOR_RGB2GRAY )
        #convert to float
        #img_gray =  np.float32(sonar_img_CV)   
        #img_gray /= 255
        # compute coefficients 
        #mode = 'db1'
        #coeffs=pywt.wavedec2(img_gray, 'db1')

        #Process Coefficients
        #coeffs_H=list(coeffs)  
        #coeffs_H[0] *= 0;  

        # reconstruction
        #imArray_H=pywt.waverec2(coeffs_H, mode)
        #imArray_H *= 255
        #imArray_H =  np.uint8(imArray_H)
        #Display result
        #cv2.imshow('image',imArray_H)
        #cv2.waitKey(0)
        if len(corners_blue) > 0:
            cv2.putText(sonar_img_flipped,text_ang,(19,31),cv2.FONT_HERSHEY_SIMPLEX,1,white,2)
            cv2.putText(sonar_img_flipped,text_err,(19,110),cv2.FONT_HERSHEY_SIMPLEX,1,white,2)
            print('x_cent ='+str(x_cent_b))
        else:
            print('Nessuna pipe rilevata!!!')
        #print(matrix_of_int)
        #cv2.imshow("Sonar Image in coordinate cartesiane",sonar_img_cartesian)
        #cv2.waitKey(1)

        cv2.imshow("Sonar Image",sonar_img_CV)
        cv2.waitKey(1)

        #cv2.imshow("Sonar Image flipped",sonar_img_flipped)
        #cv2.waitKey(1)

def sonar2vehicle(xs,ys,zs,x0,y0,z0,theta):
    XS = np.array([[xs],[ys],[zs]])
    X0 = np.array([[x0],[y0],[z0]])

    R_theta = np.array([[math.cos(math.radians(theta)),0,math.sin(math.radians(theta))],[0,1,0],[-math.sin(math.radians(theta)),0,math.cos(math.radians(theta))]])
    XV = X0 + np.dot(R_theta,XS)

    return XV[0],XV[1],XV[2]
# -------   INITIALIZE INSTANCE  -------
if __name__ == '__main__':
    node = sonar_detect()
    node.loop()