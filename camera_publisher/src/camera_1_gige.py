import rospy 
from sensor_msgs.msg import Image, CompressedImage 
from cv_bridge import CvBridge 
import cv2 
import numpy as np
import cvb
from cvb import DeviceFactory, DiscoverFlags, AcquisitionStack, ImageStream

  
def publish_message():
  # Creating the topic:
  pub = rospy.Publisher('/camera_topic_1', CompressedImage, queue_size=10)
     
  # creating ndoe
  rospy.init_node('camera_1', anonymous=True)
     
  # Go through the loop 10 times per second
  rate = rospy.Rate(28) # 10hz
     
  devices = DeviceFactory.discover_from_root(DiscoverFlags.IgnoreVins)
  print(devices)

  for i in range(len(devices)):
    print(i)
    print(devices[i].access_token[186:195])
    if devices[i].access_token[186:195] == "10.5.0.21":
      device_index = i

  device = DeviceFactory.open(devices[device_index].access_token, AcquisitionStack.GenTL)
  
  print(device)

  stream = device.stream(ImageStream)
  nodes = device.node_maps["Device"]
  pixel_format = nodes["Std::PixelFormat"]
  pixel_format.value = "BayerGB8"
  # stream.register_managed_flow_set_pool(3)

  rospy.loginfo('publishing video frame')

  stream.start()
  rospy.sleep(2)
  # While ROS is still running.
  while not rospy.is_shutdown():
      im, status, node_maps = stream.wait_for(time_span=20)
      # print(f"status {status}")
      # print(cvb.as_array(im))
      # if cvb.as_array(im) == []:
      #   print("Reseting Camera")
      #   for i in range(len(devices)):
      #     print(i)
      #     print(devices[i].access_token[186:195])
      #     if devices[i].access_token[186:195] == "10.5.0.21":
      #       device_index = i

      #   device = DeviceFactory.open(devices[device_index].access_token, AcquisitionStack.GenTL)
        
      #   print(device)

      #   stream = device.stream(ImageStream)
      #   nodes = device.node_maps["Device"]
      #   pixel_format = nodes["Std::PixelFormat"]
      #   pixel_format.value = "BayerGB8"
      #   # stream.register_managed_flow_set_pool(3)

      #   rospy.loginfo('publishing video frame')

      #   stream.start()
      image = cvb.as_array(im)
      og_frame = cv2.cvtColor(image, cv2.COLOR_BayerGB2RGB)

      og_frame = cv2.resize(og_frame,(320,240),fx=0,fy=0, interpolation = cv2.INTER_CUBIC)
      og_frame = cv2.rotate(og_frame, cv2.ROTATE_180)

      msg = CompressedImage()
      msg.header.stamp = rospy.Time.now()
      msg.format = "png"
      msg.data = np.array(cv2.imencode('.png', og_frame)[1]).tostring()
            
      pub.publish(msg)
             
      # Sleep just enough to maintain the desired rate
      rate.sleep()
  
      if rospy.is_shutdown():
        stream.abort() 
        rospy.sleep(2)
        print("Shutting down camera!")

  print("Dropping out")
  with im:
      pass
  stream.abort()
  if rospy.is_shutdown():
        stream.abort() 
        rospy.sleep(2)
        print("Shutting down camera!")

if __name__ == '__main__':
  try:
    publish_message()
  except rospy.ROSInterruptException:
    pass

