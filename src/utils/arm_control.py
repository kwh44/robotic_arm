#!/usr/bin/python3

# taken from https://github.com/AcutronicRobotics/mara_examples

import rclpy
from multiprocessing import Process
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from hrim_actuator_rotaryservo_msgs.msg import GoalRotaryServo
from hrim_actuator_gripper_srvs.srv import ControlFinger

class Gripper(Node):

    def __init__(self):
        super().__init__('mara_minimal_client')
        # Create a client for service "/hrim_actuation_gripper_000000000004/goal"
        self.client = self.create_client(ControlFinger, "/hrim_actuator_gripper_000000000004/fingercontrol")
        # Wait for service to be avaiable before calling it
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        # Create request with the same type as the service, ControlFinger
        self.req = ControlFinger.Request()

    def send_request(self):
        self.future = self.client.call_async(self.req)

    @staticmethod
    def run(cmd_queue):
        rclpy.init(args=None)
        node = Gripper()
        node.req.goal_velocity = 9999.
        while True:
            node.req.goal_angularposition = cmd_queue.get()
            # Call service and spin
            node.send_request()
            rclpy.spin_until_future_complete(node, node.future)


class JointMove(Node):
    def __init__(self, x, y, cmd_queue):
        # Initialize Node with name "mara_minimal_publisher"
        super().__init__('mara_minimal_publisher' + "_" + str(x) + "_" + str(y))
        # Create a publisher on topic "/hrim_actuation_servomotor_000000000001/goal_axis1"
        # !TODO one class controls all six joints
        self.pub_ = self.create_publisher(GoalRotaryServo, '/hrim_actuator_rotaryservo_00000000000' + str(x) + '/goal_axis' + str(y),
                                                qos_profile=qos_profile_sensor_data)
        # Create message with the same type as the topic, GoalRotaryServo
        self.msg = GoalRotaryServo()
        # Create a timer to publish the messages periodically
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.cmd_queue = cmd_queue
        self.cmd = None


    def timer_callback(self):
        # Fill message content
        if self.cmd is None or not self.cmd_queue.empty():
            self.cmd = self.cmd_queue.get()
        self.msg.position = self.cmd  * 3.1416/180 # Position to rads
        self.msg.velocity = 30.                      # Velocity in rads/s
        self.msg.control_type = 4                     # Position and velocity control
        # Publish message!
        self.pub_.publish(self.msg)

    @staticmethod
    def run(x, y, cmd_queue):
        rclpy.init(args=None)
        minimal_publisher = JointMove(x, y, cmd_queue)
        rclpy.spin(minimal_publisher)
        minimal_publisher.destroy_node()
        rclpy.shutdown()



def main(args=None):

    # cmd_queue = Queue()
    # cmd_queue.put(0.7)
    # cmd_queue.put(0.05)
    #
    # #processes = [Process(target=JointMove.run, args=(i, j)) for i in range(1, 4) for j in range(1, 3)]
    # processes = []
    # processes.append(Process(target=Gripper.run, args=(cmd_queue,)))
    #
    # for process in processes:
    #     process.start()
    #
    # for process in processes:
    #     process.join()
    #
    #
    pass
if __name__ == '__main__':
    main()
