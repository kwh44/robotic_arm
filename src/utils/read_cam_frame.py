import rclpy
from rclpy.node import Node
import rclpy.qos as qos
import cv2
import numpy as np
from sensor_msgs.msg import Image
from threading import Lock

class CameraRead(Node):
    def __init__(self):
        super().__init__('CameraRead')
        self.__window_name = "img"
        self._qos = qos.QoSProfile(history=1, depth=1)
        self.sub = self.create_subscription(Image, '/rs_camera/rs_d435/image_raw', self.msg_callback, qos_profile=self._qos)
        self.last_observation = None
        self.mtx = Lock()

    def msg_callback(self, m):
        np_img = np.reshape(m.data, (m.height, m.width, 3)).astype(np.uint8)
        np_img = cv2.cvtColor(np_img, cv2.COLOR_BGR2RGB)
        self.mtx.acquire()
        self.last_observation = np_img
        self.mtx.release()

    def get_frame(self):
        self.mtx.acquire()
        frame = self.last_observation
        self.mtx.release()
        return frame

    @staticmethod
    def run(node):
        rclpy.init(args=None)
        rclpy.spin(node)

        node.destroy_node()
        rclpy.shutdown()
