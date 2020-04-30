#!/usr/bin/env python2
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

#Web scraping:
import matplotlib.pyplot as plt
import requests
import numpy
import PIL
from io import BytesIO

#Firefox login into the webserver:
from selenium import webdriver
from selenium.webdriver.common.keys import Keys
from selenium.common.exceptions import WebDriverException
#from selenium.webdriver.support import expected_conditions as EC
#from selenium.webdriver.common.by import By
#from selenium.common.exceptions import TimeoutException
#from selenium.webdriver.support.ui import WebDriverWait
rospy.init_node('flir_web_scraping_driver')
#For headless firefox:
import os
os.environ['MOZ_HEADLESS'] = '1' 
<<<<<<< HEAD
#Threading:
import threading
thread_time = rospy.Time.now()
login_timeout = True
=======


>>>>>>> 3452f2f9ff57787625ff418f69052ad30a31fd49


def flir_publisher():
<<<<<<< HEAD


=======
    rospy.init_node('flir_web_scraping_driver')
    #Threading:
    import threading
    thread_time = rospy.Time.now()
    login_timeout = True
>>>>>>> 3452f2f9ff57787625ff418f69052ad30a31fd49

    camera_frame = rospy.get_param('~flir_camera_frame', "flir_ax8_link")
    camera_topic = rospy.get_param('~flir_camera_topic', "flir_ax8")
    camera_ip = rospy.get_param('~flir_camera_ip', "172.16.2.10")
    global login_timeout
    
    rospy.loginfo("FLIR: Login into Web Server...")
    url = 'http://'+camera_ip
    start_thread(url,True) #Initial login
    
    bridge = CvBridge()
    pub = rospy.Publisher(camera_topic, Image, queue_size=10)
    rate = rospy.Rate(10) #the actual FPS will be 8.8/9Hz
  
    start_publish = True
    repeated_images = 0
    previous_image = numpy.empty((480,640,3))

    while not rospy.is_shutdown():

        # get image (via web scraping):
        r = requests.get(url+'/snapshot.jpg', stream=True) #request jpg img stream
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
            if repeated_images > 4:
                rospy.logerr("FLIR: Camera not updating the image. Last login was %ld secs ago.",rospy.Time.now().secs-thread_time.secs)
                if login_timeout==True:
                   rospy.loginfo("FLIR: A new login is currently underway.") 
                   while login_timeout:
                      rospy.sleep(0.1)
                else:
                   rospy.logwarn("FLIR: Will enforce a new login.")
                   start_thread(url,True) #force a new login and wait for thread to join
        else:
            #ROS stuff (publish the most recent image):
            pub_img = bridge.cv2_to_imgmsg(open_cv_image, encoding="bgr8")
            pub_img.header.stamp = rospy.Time.now()
            pub_img.header.frame_id = camera_frame
            pub.publish(pub_img)
            previous_image = open_cv_image
            repeated_images = 0
            
        # Login timeouts - start thread to login again every 10 secs
        if login_timeout == False and (rospy.Time.now().secs-thread_time.secs)>10.0: 
            #rospy.logwarn("Login Timeout")
            start_thread(url,False)  #do not wait for thread to join
            login_timeout = True     #prevent from repeating the condition and open a new thread

        if start_publish: #initial msg:
            rospy.loginfo("FLIR: Publishing images on topic \"" + camera_topic + "\"")
            start_publish=False
        
        rate.sleep()
        
        
def firefox_login_thread(url_str): #login (30 sec timeout)
    browser = webdriver.Firefox()
    browser.get(url_str) 
    username = browser.find_element_by_id("login_input_username")
    username.send_keys("admin") 
    password = browser.find_element_by_id("login_input_password")
    password.send_keys("admin") 
    password.submit() 
    
    #Check if login was successful and retry if needed:
    no_sucess=True
    sleep_time=5.0
    
    while no_sucess:
       rospy.sleep(sleep_time) #first sleep long
       try:
          if "success" in browser.page_source: #able to login successfully
             no_sucess=False
       except WebDriverException:
          rospy.logwarn("FLIR: Not able to log in momentarily. Will try again immediately.")
          sleep_time=0.25 #sleep just a small bit to try again
    
    if (sleep_time<5.0): #only print when following a warning
        rospy.loginfo("FLIR: Successfully logged in again!")
    
    global thread_time
    thread_time = rospy.Time.now()
    browser.close()
    global login_timeout
    login_timeout = False
    
def start_thread(url_str, join):
    firefox_thread = threading.Thread(target=firefox_login_thread, args=(url_str,))
    firefox_thread.start()   
    if join:
        firefox_thread.join() #Block and wait for the thread to finish (we'll have a gap in the data)
        rospy.loginfo("FLIR: Successfully logged into Web Server!")

if __name__ == '__main__':
    try:
        flir_publisher()
    except rospy.ROSInterruptException:
        pass
