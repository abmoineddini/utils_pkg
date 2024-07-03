#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
from src.utils.ros import params


# Finding the index of the corresponding camera
config = params.ParameterSet(namespace="sensor_manager")
index = config['camera_3'].get('index')  

def publish_message(index):
  # Creating the topic:
  pub = rospy.Publisher('/camera_topic_1', CompressedImage)
     
  # creating ndoe
  rospy.init_node('camera_1', anonymous=False)
     
  # Go through the loop 10 times per second
  rate = rospy.Rate(50) # 10hz
     
  # Create a VideoCapture object
  # to be set by the camera config file
  cap = cv2.VideoCapture(index)
  rospy.loginfo('publishing video frame')
  # While ROS is still running.
  while not rospy.is_shutdown():
     
      ret, frame = cap.read(index)
      # print("index is :{}".format(index))
      frame = cv2.resize(frame,(320,240),fx=0,fy=0, interpolation = cv2.INTER_CUBIC)
         
      if ret == True:
        # Print debugging information to the terminal
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpeg', frame)[1]).tostring()
             
        pub.publish(msg)
             
      # Sleep just enough to maintain the desired rate
      rate.sleep()
         
if __name__ == '__main__':
  try:
    publish_message(index)
  except rospy.ROSInterruptException:
    pass