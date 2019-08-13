#!/usr/bin/env python2
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import matplotlib.pyplot as plt
import requests
import numpy
import PIL
from io import BytesIO


def flir_publisher():
    rospy.init_node('flir_web_scraping_driver')

    camera_frame = rospy.get_param('~flir_camera_frame', "flir_ax8_link")
    camera_topic = rospy.get_param('~flir_camera_topic', "flir_ax8")
    camera_ip = rospy.get_param('~flir_camera_ip', "192.168.1.154")

    url = "http://"+camera_ip+"/snapshot.jpg"

    bridge = CvBridge()
    pub = rospy.Publisher(camera_topic, Image, queue_size=10)
    rate = rospy.Rate(10) #the actual FPS will be 8.8/9Hz
    
    start_publish = True
    repeated_images = 0
    previous_image = numpy.empty((480,640,3))

    while not rospy.is_shutdown():

        # get image (via web scraping):
        r = requests.get('http://'+camera_ip+'/snapshot.jpg', stream=True)
    	im = PIL.Image.open(BytesIO(r.content))
        pil_image = im.convert('RGB')
        open_cv_image = numpy.array(pil_image) 
        
        # Convert RGB to BGR:
        open_cv_image = open_cv_image[:, :, ::-1].copy() 

        #check the image on OpenCV:
        #cv2.imshow('Flir AX8 Feed', open_cv_image)
        #cv2.waitKey(1)
        
        if numpy.array_equal(open_cv_image, previous_image):
            # Acquisition is 8.8Hz, Max publishing is 10 Hz. This condition will be common (at least once/sec)
            repeated_images+=1 #Same img as before
            #rospy.loginfo("same image")
            if repeated_images > 4:
                rospy.logerr("The Flir camera is not updating the image. Are you logged in?")
        else:
            #ROS stuff (publish last image):
            pub_img = bridge.cv2_to_imgmsg(open_cv_image, encoding="bgr8")
            pub_img.header.stamp = rospy.Time.now()
            pub_img.header.frame_id = camera_frame
            pub.publish(pub_img)
            previous_image = open_cv_image
            repeated_images = 0

        if start_publish:
            rospy.loginfo("Publishing images on topic \"" + camera_topic + "\"")
            start_publish=False

        rate.sleep()

if __name__ == '__main__':
    try:
        flir_publisher()
    except rospy.ROSInterruptException:
        pass
