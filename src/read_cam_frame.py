
import rclpy
from rclpy.node import Node
import rclpy.qos as qos
import cv2
import numpy as np
from sensor_msgs.msg import Image

class CameraRead(Node):
    def __init__(self):
        super().__init__('CameraRead')
        self.__window_name = "img"
        self._qos = qos.QoSProfile(history=1, depth=1)
        self.sub = self.create_subscription(Image, '/rs_camera/rs_d435/image_raw', self.msg_callback, qos_profile=self._qos)

    def msg_callback(self, m):
        print("received image stamp {}".format(m.header.frame_id))
        np_img = np.reshape(m.data, (m.height, m.width, 3)).astype(np.uint8)
        np_img = cv2.cvtColor(np_img, cv2.COLOR_BGR2RGB)
        cv2.imshow(self.__window_name, np_img)
        cv2.waitKey(1)
        #

def main():
    rclpy.init(args=None)
    node = CameraRead()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()