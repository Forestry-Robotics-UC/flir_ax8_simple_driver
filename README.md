# flir_ax8_simple_driver

### ver0.2 (David Portugal)
Simple driver for the FLIR AX8 Thermal Camera using OpenCV and RTSP to stream the camera's image, and publishing it into a sensor_msgs/Image topic on ROS

Tested on Ubuntu 18.04 and ROS Melodic

## Important

Make sure that you have the correct camera IP when running the launch file.
You'll need to ```sudo apt install python-pip```, in case you don't have pip installed.

## Compling

```
cd your_work_space
catkin_make 
```

## Example Usage

### flir_ax8_simple_driver

**Parameters**

`flir_camera_ip` (`string`, `default: 169.254.64.1`)

By default, the IP address of the device is 169.254.64.1.

`flir_camera_frame` (`string`, `default: flir_ax8`)

The frame ID entry for the messages.

`flir_camera_topic` (`string`, `default: camera/flir_ax8`)

The topic to publish the messages.

`flir_encoding` (`string`, `default: mpeg4`)

The encoding type for the streamed images. Possible options are: "avc", "mpeg4" and "mjpg".

`flir_text_overlay` (`string`, `default: off`)

Option to remove the overlaid text from FLIR. Possible options are: "on" and "off".

**Node**

```
roslaunch flir_ax8_simple_driver flir_ax8_simple_driver.launch
```

Feel free to play with the parameters on the launch file to suit your setup.
