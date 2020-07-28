#!/usr/bin/python3

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
    def run(angularposition):
        rclpy.init(args=None)

        node = Gripper()
        
        # TODO! pop goal agular position from queue 

        node.req.goal_angularposition = angularposition

        # Call service and spin
        node.send_request()
        rclpy.spin_until_future_complete(node, node.future)

        # Analyze the result
        if node.future.result() is not None:
            node.get_logger().info('Goal accepted: %d: ' % node.future.result().goal_accepted)
        else:
            node.get_logger().error('Exception while calling service: %r' % node.future.exception())

        node.destroy_node()
        rclpy.shutdown()


class JointMove(Node):

    def __init__(self, x, y):
        # Initialize Node with name "mara_minimal_publisher"
        super().__init__('mara_minimal_publisher' + "_" + str(x) + "_" + str(y))

        # Create a publisher on topic "/hrim_actuation_servomotor_000000000001/goal_axis1"
        self.pub_ = self.create_publisher(GoalRotaryServo, '/hrim_actuator_rotaryservo_00000000000' + str(x) + '/goal_axis' + str(y),
                                                qos_profile=qos_profile_sensor_data)

        # Create message with the same type as the topic, GoalRotaryServo
        self.msg = GoalRotaryServo()

        # Create a timer to publish the messages periodically
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.position_deg = 30.
        self.i = 0 # Iteration counter


    def timer_callback(self):

        # TODO! pop message content from queue

        # Fill message content
        self.position_deg = 30 * (-1 * (self.i % 2))
        self.msg.position = self.position_deg * 3.1416/180 # Position to rads
        self.msg.velocity = 20.                      # Velocity in rads/s
        self.msg.control_type = 4                     # Position and velocity control

        # Publish message!
        self.pub_.publish(self.msg)

        # Log
        self.get_logger().info("Iteration number: {}".format(self.i))
        self.i += 1


    @staticmethod
    def run(x, y):

        rclpy.init(args=None)

        minimal_publisher = JointMove(x, y)

        rclpy.spin(minimal_publisher)

        minimal_publisher.destroy_node()

        rclpy.shutdown()



def main(args=None):


    processes = [Process(target=JointMove.run, args=(i, j)) for i in range(1, 4) for j in range(1, 3)]
    
    processes.append(Process(target=Gripper.run, args=(0.5,)))
    
    for process in processes:
        process.start()

    for process in processes:
        process.join()



if __name__ == '__main__':
    main()
