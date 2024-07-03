#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from src.utils.ros import params


class camera(object):

    def __init__(self, index):
        self.pub = rospy.Publisher(f'/camera_topic_4', Image, queue_size=10)
        self.cap = cv2.VideoCapture(index)
        rospy.loginfo("Camera 4 functional")
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.ret, self.frame = self.cap.read()
            self.bridge = CvBridge()
            self.img_array = np.asarray(self.frame)
        
            try:
                self.img_msg = self.bridge.cv2_to_imgmsg(self.frame, "bgr8")
            except CvBridgeError as e:
                print(e)
            
            self.pub.publish(self.img_msg)
            rate.sleep()


if __name__ == "__main__":
    config = params.ParameterSet(namespace="sensor_manager")
    rospy.init_node("camera_4", anonymous=False)
    index = config['camera_4'].get('index')
    # print("index is :{}".format(index))
    camera = camera(index=index)
    rospy.spin()
