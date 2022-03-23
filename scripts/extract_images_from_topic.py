#! /usr/bin/python
# Copyright (c) 2015, Rethink Robotics, Inc.

# Using this CvBridge Tutorial for converting
# ROS images to OpenCV2 images
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

# Using this OpenCV2 tutorial for saving Images:
# http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html

# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2

# Instantiate CvBridge
bridge = CvBridge()

def image_callback(msg):
    print("Received an image!")
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError, e:
        print(e)
    else:
        # Save your OpenCV2 image as a jpeg 
        seq = msg.header.seq        
        cv2.imwrite('camera_rear_left_'+str(seq)+'.jpeg', cv2_img)
        rospy.sleep(1)

def right_image_callback(msg):
    print("Received an image!")
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError, e:
        print(e)
    else:
        # Save your OpenCV2 image as a jpeg 
        seq = msg.header.seq        
        cv2.imwrite('camera_rear_right_'+str(seq)+'.jpeg', cv2_img)
        rospy.sleep(1)

def main():
    rospy.init_node('image_listener')
    # Define your image topic
    image_topic = "/camera_rear_left/color/image_raw/output"
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, image_callback)
    
    # Define your image topic
    right_image_topic = "/camera_rear_right/color/image_raw/output"
    # Set up your subscriber and define its callback
    rospy.Subscriber(right_image_topic, Image, right_image_callback)
 
    # Spin until ctrl + c
    rospy.spin()

if __name__ == '__main__':
    main()
