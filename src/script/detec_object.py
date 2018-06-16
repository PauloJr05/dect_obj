#!/usr/bin/env python

from __future__ import print_function

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError


class detec_object:
    def __init__(self):
        self.pub = rospy.Publisher ('object_topic', String, queue_size=10)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/raspicam_node/image', Image, self.callback)
    
	

    def detec_blue(self, image):
	
	#cap.set(15, 0.1)

        rangeBlueMax = np.array([255, 50, 50]) # B ,G, R
        rangeBlueMin = np.array([51, 0, 0])
        mask = cv2.inRange(image, rangeBlueMin, rangeBlueMax) #mascara que identifica o objeto

        #filter
        kernel = np.ones((5,5), np.uint8)
	dilation = cv2.dilate(mask,kernel,iterations = 1)
        opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
	closing = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)


	


        #x,y,w,h = cv2.boundingRect(opening)
        #cv2.rectangle(image, (x,y), (x+y, y+h), (0, 255, 0), 3)
        #cv2.circle(image, (x+w/2,y+h/2),5,(0,0,255),-1)

        cv2.namedWindow('Window',cv2.WINDOW_NORMAL)
        cv2.imshow('Window',opening)
	cv2.namedWindow('Win',cv2.WINDOW_NORMAL)
        cv2.imshow('Win',dilation)
	cv2.namedWindow('Wind',cv2.WINDOW_NORMAL)
        cv2.imshow('Wind',closing)
	cv2.namedWindow('mask',cv2.WINDOW_NORMAL)
        cv2.imshow('mask',image)
        cv2.waitKey(5)
        
        X_str = ('X: {}'.format(100))
        self.pub.publish(X_str)

        
    def callback(self, data):
        try:
	    #data.set(15,0.1)
            cap = self.bridge.imgmsg_to_cv2(data,"bgr8") # converte uma imagem de ROS para openCV
            self.detec_blue(cap)
        except CvBridgeError as error:
            print(error)

       
        

def main(args):
     ic = detec_object()
     rospy.init_node('detec_object', anonymous=True)
     
     try:
        rospy.spin()
     except KeyboardInterrupt:
        print("Shutting down")
     cv2.destroyAllWindows()
if __name__ == '__main__':
    main(sys.argv)
        
        

        


        
        

