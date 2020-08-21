import gym
import rclpy
import cv2
import numpy as np
from gym import spaces
from sensor_msgs.msg import Image
import rclpy.qos as qos
from rclpy.qos import qos_profile_sensor_data
from hrim_actuator_rotaryservo_msgs.msg import GoalRotaryServo
from hrim_actuator_gripper_srvs.srv import ControlFinger


class RoboticArm(gym.Env):
    """
    Gazebo Robotic Arm Environment that follows gym interface.
    """

    def __init__(self):

        super(RoboticArm, self).__init__()
        # 6 joint angles and gripper angular position
        self.action_space = spaces.Box(low=np.array([-145.0, -180.0, -145.0, -90.0, -120.0, -90.0, 0.0]),
                                       high=np.array([145.0, 180.0, 145.0, 90.0, 120.0, 90.0, 0.87]), dtype=np.float32)
        # image from realsense camera 640x480
        self.observation_space = spaces.Box(
            low=0,
            high=255,
            shape=(480, 640, 3),
            dtype=np.uint8
        )
        rclpy.init(args=None)
        self.__start_video_feed()
        self.__start_arm_control()

    def reset(self):
        """
        reset() called at the beginning of an episode, it returns an observation
        """
        # set joints and gripper to default position
        # move coke and beer to start position
        pass

    def step(self, action):
        """
        step(action) called to take an action with the environment, it returns the next observation,
        the immediate reward, whether the episode is over and additional information
        """
        # take action and get observation
        self.__publish_arm_cmds(action)
        observation = self.__get_observation()
        # is episode complete
        done = False
        # reward
        # + 10 if beer is on the right side or coke is on left side from the arm
        #
        reward = None

        # debugging information
        info = {}
        pass

    def close(self):
        pass

    def __get_observation(self):
        # execute camera feed callback
        rclpy.spin_once(self.camera_feed_node)
        return self.last_observation

    def __publish_arm_cmds(self, action):
        # set joint angle
        for i in range(6):
            self.arm_cmd_msgs[i].position = action[i] * 3.1416 / 180
            # publish three times to the joint angle topic
            for i in range(3):
                self.arm_cmd_nodes_pubs[i][1].publish(self.arm_cmd_msgs[i])
        # set gripper request
        self.arm_cmd_msgs[-1].goal_angularposition = action[6]
        while not self.arm_cmd_nodes_pubs[-1][1].wait_for_service(timeout_sec=1.0):
            continue
        goal_accepted = False
        while not goal_accepted:
            gripper_future = self.arm_cmd_nodes_pubs[-1][1].call_async(self.arm_cmd_msgs[-1])
            rclpy.spin_until_future_completes(self.arm_cmd_nodes_pubs[-1][0], gripper_future)
            if gripper_future.result() is not None:
                if gripper_future.result().goal_accepted:
                    goal_accepted = True


    def __msg_callback(self, m):
        np_img = np.reshape(m.data, (m.height, m.width, 3)).astype(np.uint8)
        np_img = cv2.cvtColor(np_img, cv2.COLOR_BGR2RGB)
        self.last_observation = np_img

    def __start_video_feed(self):
        self.camera_feed_node = rclpy.create_node("mara_camera_feed_node")
        self._qos = qos.QoSProfile(history=1, depth=1)
        self.sub = self.camera_feed_node.create_subscription(Image, '/rs_camera/rs_d435/image_raw', self.__msg_callback,
                                                             qos_profile=self._qos)

    def __start_arm_control(self):
        # nodes and publishers for six angle joints
        self.arm_cmd_nodes_pubs = []
        for x in range(1, 4):
            for y in range(1, 3):
                node = rclpy.create_node("mara_arm_cmd_" + str(x) + "_" + str(y))
                self.arm_cmd_nodes_pubs.append([node, node.create_publisher(GoalRotaryServo,
                                                                           '/hrim_actuator_rotaryservo_00000000000'
                                                                            + str(x) + '/goal_axis' + str(y),
                                                                           qos_profile=qos_profile_sensor_data)])
        # declare joint angle messages
        self.arm_cmd_msgs = []
        for i in range(6):
            msg = GoalRotaryServo()
            msg.position = 0.0                        # Position to rads
            msg.velocity = 30.                        # Velocity in rads/s
            msg.control_type = 4                      # Position and velocity control
            self.arm_cmd_msgs.append(msg)

        # gripper node and service client
        node = rclpy.create_node("mara_gripper_cmd")
        client = node.create_client(ControlFinger, "/hrim_actuator_gripper_000000000004/fingercontrol")
        self.arm_cmd_nodes_pubs.append([node, client])

        # declare gripper service request
        req = ControlFinger.Request()
        req.goal_velocity = 99999.
        req.goal_angularposition = 0.0
        self.arm_cmd_msgs.append(req)