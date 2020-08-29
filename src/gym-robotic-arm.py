import gym
import rclpy
import cv2
import numpy as np
import rclpy.qos as qos
from gym import spaces
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from gazebo_msgs.srv import SpawnEntity
from gazebo_msgs.srv import DeleteEntity
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
        self.__first_episode = True
        self.target_obj1_pos = np.array([0.5, -0.5, 0.0]) # coke_can
        self.target_obj2_pos = np.array([-0.5, -0.5, 0.0]) # beer

        rclpy.init(args=None)
        self.__start_video_feed()
        self.__start_arm_control()
        self.__init_spawn()
        self.__init_delete()

    def reset(self):
        """
        reset() called at the beginning of an episode, it returns an observation
        """
        if self.__first_episode:
            self.__spawn()
            self.__first_episode = False
        else:
            # delete coke_can and beer models
            self.__delete()
            # spawn them in the origin position
            self.__spawn()
        # set joint to default position
        action_cmd = [0., 0., 0., 0., 45., 0., 0.8]
        self.__publish_arm_cmds(action_cmd)
        return self.__get_observation()

    def step(self, action):
        """
        step(action) called to take an action with the environment, it returns the next observation,
        the immediate reward, whether the episode is over and additional information
        """
        # take action and get observation
        self.__publish_arm_cmds(action)
        observation = self.__get_observation()
        # is episode complete
        done = self.is_episode_over()
        # reward
        # + 10 if beer is on the right side or coke is on left side from the arm
        reward = self.reward_function()
        # debugging information
        info = {}
        return observation, reward, done, info

    def close(self):
        pass

    def __init_spawn(self):
        self.entity_spawner = rclpy.create_node("entity_spawner")
        self.spawn_client = self.entity_spawner.create_client(SpawnEntity, "/spawn_entity")
        if not self.spawn_client.service_is_ready():
            self.spawn_client.wait_for_service()

    def __init_delete(self):
        self.del_node = rclpy.create_node("entity_delete")
        self.del_client = self.del_node.create_client(DeleteEntity, "/delete_entity")
        if not self.del_client.service_is_ready():
            self.del_client.wait_for_service()

    def __spawn(self):
        models = [["beer", "/root/robotic_arm/models/beer/model.sdf", (-0.18, 0.5, 0.)],
                  ["coke_can", "/root/robotic_arm/models/coke_can/model.sdf", (0.24, 0.5, 0.)]]
        for model in models:
            request = SpawnEntity.Request()
            request.name = model[0]
            request.xml = open(model[1], 'r').read()
            request.robot_namespace = "spawned_object"
            request.initial_pose.position.x, request.initial_pose.position.y, request.initial_pose.position.z = model[2]
            done = False
            while not done:
                future = self.spawn_client.call_async(request)
                rclpy.spin_until_future_complete(self.entity_spawner, future)
                if future.result().success:
                    done = True

    def __delete(self):
        models = ["coke_can", "beer"]
        for model in models:
            request = DeleteEntity.Request()
            request.name = model
            done = False
            while not done:
                future = self.del_client.call_async(request)
                rclpy.spin_until_future_complete(self.del_node, future)
                if future.result().success:
                    done = True

    def reward_function(self):
        reward = 0.0
        obj1_pos, obj2_pos = self.__get_object_pos()
        reward += self.__distance(obj1_pos, self.target_obj1_pos)
        reward += self.__distance(obj2_pos, self.target_obj2_pos)
        return reward

    def is_episode_over(self):
        dist = 0.1
        obj1_pos, obj2_pos = self.__get_object_pos()
        return self.__distance(obj1_pos, self.target_obj1_pos) < dist and self.__distance(obj2_pos,
                                                                                          self.target_obj2_pos) < dist

    def __distance(self, p1, p2):
        return np.sqrt(np.sum((p1 - p2) ** 2, axis=0))

    def __get_object_pos(self):
        rclpy.spin_once(self.object_type_1_node)
        rclpy.spin_once(self.object_type_2_node)
        return self.last_object_type_1_pos, self.last_object_type_2_pos

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
            rclpy.spin_until_future_complete(self.arm_cmd_nodes_pubs[-1][0], gripper_future)
            if gripper_future.result() is not None:
                if gripper_future.result().goal_accepted:
                    goal_accepted = True

    def __cam_callback(self, m):
        np_img = np.reshape(m.data, (m.height, m.width, 3)).astype(np.uint8)
        np_img = cv2.cvtColor(np_img, cv2.COLOR_BGR2RGB)
        self.last_observation = np_img

    def __start_video_feed(self):
        self.camera_feed_node = rclpy.create_node("mara_camera_feed_node")
        self._qos = qos.QoSProfile(history=1, depth=1)
        self.sub = self.camera_feed_node.create_subscription(Image, '/rs_camera/rs_d435/image_raw', self.__cam_callback,
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
            msg.position = 0.0  # Position to rads
            msg.velocity = 30.  # Velocity in rads/s
            msg.control_type = 4  # Position and velocity control
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

    def __object_type_1_callback(self, m):
        self.last_object_type_1_pos = np.array([m.x, m.y, m.z])

    def __object_type_2_callback(self, m):
        self.last_object_type_2_pos = np.array([m.x, m.y, m.z])

    def __start_entity_state(self):
        self.object_type_1_node = rclpy.create_node("mara_object_type_1_node")
        self.sub_object_type_1 = self.object_type_1_node.create_subscription(Point,
                                                                             '/spawned_object/coke_can_entity_state',
                                                                             self.__object_type_1_callback,
                                                                             qos_profile=self._qos)
        self.object_type_2_node = rclpy.create_node("mara_object_type_2_node")
        self.sub_object_type_2 = self.object_type_2_node.create_subscription(Point, '/spawned_object/beer_entity_state',
                                                                             self.__object_type_2_callback,
                                                                             qos_profile=self._qos)
