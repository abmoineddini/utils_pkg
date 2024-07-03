# from cv_bridge import CvBridge 
import cv2 
import numpy as np
import rclpy
import cvb
from cvb import DeviceFactory, DiscoverFlags, AcquisitionStack, ImageStream

class Camera(Node):
    def __init__(self):
        super().__init__("gige_camera_publisher")
        self.publisher = self.create_publisher(CompressedImage, "camera_topic", 10)
        self._timer = self.create_timer(1/28, self.callback)
        self.devices = DeviceFactory.discover_from_root(DiscoverFlags.IgnoreVins)
        print(self.devices)

        for i in range(len(self.devices)):
            print(i)
            print(self.devices[i].access_token[186:199])
            if self.devices[i].access_token[186:199] == "192.168.1.112":
                self.device_index = i

        self.device = DeviceFactory.open(self.devices[self.device_index].access_token, AcquisitionStack.GenTL)

        print(self.device)
        

        self.stream = self.device.stream(ImageStream)
        self.nodes = self.device.node_maps["Device"]
        self.pixel_format = self.nodes["Std::PixelFormat"]
        self.pixel_format.value = "BayerGB8"

        self.get_logger().info("camera publishing ready")
        
        self.stream.start()
        # self.rate = Node.create_rate(frequency=0.5)
        # self.rate.sleep(2)    
    
    def callback(self):
      im, status, node_maps = self.stream.wait_for(time_span=20)
      image = cvb.as_array(im)
      og_frame = cv2.cvtColor(image, cv2.COLOR_BayerGB2RGB)

      og_frame = cv2.resize(og_frame,(320,240),fx=0,fy=0, interpolation = cv2.INTER_CUBIC)
      og_frame = cv2.rotate(og_frame, cv2.ROTATE_180)

      msg = CompressedImage()
      msg.format = "png"
      msg.data = np.array(cv2.imencode('.png', og_frame)[1]).tostring()
            
      self.publish.publish(msg)
    
    def shutdown(self):
       self.stream.abort()
       self.rate.sleep()


def main(args=None):
    rclpy.init(args=args)
    node = Camera()
    rclpy.spin(node)

    node.shutdown()
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()