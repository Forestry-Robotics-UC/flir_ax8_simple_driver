#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>


int main(int argc, char **argv){
  ros::init(argc, argv, "flir_ax8_simple_driver_rtsp");
  ros::NodeHandle n_p("~");
  ros::NodeHandle n;

  std::string flir_camera_frame;
  n_p.param<std::string>("flir_camera_frame", flir_camera_frame, "flir_ax8_link");
  std::string flir_camera_topic;
  n_p.param<std::string>("flir_camera_topic", flir_camera_topic, "flir_ax8");
  std::string flir_camera_ip;
  n_p.param<std::string>("flir_camera_ip", flir_camera_ip, "192.168.1.154");
  std::string flir_encoding;
  n_p.param<std::string>("flir_encoding", flir_encoding, "mpeg4");
  std::string flir_text_overlay;
  n_p.param<std::string>("flir_text_overlay", flir_text_overlay, "off");

  ros::Publisher img_pub = n.advertise<sensor_msgs::Image>(flir_camera_topic, 100);
  
  cv::VideoCapture cap( "rtsp://"+flir_camera_ip+"/"+flir_encoding+"?overlay="+flir_text_overlay );
  cap.set(CV_MAX_SOBEL_KSIZE, 1); //doesn't seem to make a difference...

  if(!cap.isOpened()){ // check if we succeeded
  	   ROS_ERROR("Could not open FLIR camera capture. Check if camera is connected and if you have the right IP.");
       return -1;
   }

  cv::Mat frame;
  ros::Rate loop_rate(20); //just an upper limit (should publish at around ~9Hz)
  bool start_publish=true;

  while(ros::ok()){

    cap >> frame; 
  	cv_bridge::CvImage out_msg;
	  out_msg.header.stamp = ros::Time::now();
	  out_msg.header.frame_id = flir_camera_frame;
	  out_msg.encoding = sensor_msgs::image_encodings::BGR8; //"passtrough"
	  out_msg.image    = frame;
	  img_pub.publish(out_msg.toImageMsg()); //publish full color image
	  if (start_publish){
		  ROS_INFO("Publishing images on topic \"%s\"",flir_camera_topic.c_str());
		  start_publish=false;
	  }

	  ros::spinOnce();
	  loop_rate.sleep();
  }

  cap.release();
  return 0;
}
