#!/usr/bin/env python2
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def flir_publisher():
    rospy.init_node('flir_ax8_simple_driver', anonymous=True)

    camera_frame = rospy.get_param('~flir_camera_frame', "flir_ax8")
    camera_topic = rospy.get_param('~flir_camera_topic', "camera/flir_ax8")
    camera_ip = rospy.get_param('~flir_camera_ip', "169.254.64.1")
    encoding = rospy.get_param('~flir_encoding', "mpeg4")       #options: avc, mpeg4 and mjpg
    overlay = rospy.get_param('~flir_text_overlay', "off")      #options: on/off (string)

    vcap = cv2.VideoCapture("rtsp://"+camera_ip+"/"+encoding+"?overlay="+overlay)

    bridge = CvBridge()

    pub = rospy.Publisher(camera_topic, Image, queue_size=10)
    rate = rospy.Rate(10) #the actual FPS will be 8.8/9Hz
    start_publish = True

    while not rospy.is_shutdown():

        #OpenCV stuff:
        ret, frame = vcap.read()
   	#cv2.imshow('Flir AX8 Feed', frame)
        #cv2.waitKey(1)

        if not ret: #unable to open capture: exit.
           rospy.logerr("Could not open Camera capture. Check if camera is connected and if you have the right IP.")
           break

        #ROS stuff:
        pub_img = bridge.cv2_to_imgmsg(frame, encoding="passthrough")
        pub_img.header.frame_id = camera_frame
        pub_img.header.stamp = rospy.Time.now()
        pub.publish(pub_img)

        if start_publish:
            rospy.loginfo("Publishing images on topic \"" + camera_topic + "\"")
            start_publish=False

        rate.sleep()

    #Release capture on exit:
    vcap.release()

if __name__ == '__main__':
    try:
        flir_publisher()
    except rospy.ROSInterruptException:
        pass
