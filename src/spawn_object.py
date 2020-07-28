# taken from https://zmk5.github.io/general/demo/2019/07/15/ros2-spawning-entity.html

import os
import sys
import rclpy
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity

def main():
    """ Main for spwaning turtlebot node """
    # Get input arguments from user
    

    # Start node
    node = rclpy.create_node("entity_spawner")

    node.get_logger().info(
        'Creating Service client to connect to `/spawn_entity`')
    client = node.create_client(SpawnEntity, "/spawn_entity")

    node.get_logger().info("Connecting to `/spawn_entity` service...")
    if not client.service_is_ready():
        client.wait_for_service()
        node.get_logger().info("...connected!")

    # Get path to the turtlebot3 burgerbot
    beer = ["~/robotic_arm/models/beer/model.sdf", "beer"]
    coke = ["~/robotic_arm/models/coke_can/model.sdf", "coke_can"]

    for sdf_file in [beer, coke]
        # Set data for request
        request = SpawnEntity.Request()
        request.name = sdf_file[0]
        request.xml = open(sdf_file[1], 'r').read()
        request.robot_namespace = "spawned_object"
        request.initial_pose.position.x = 0. #float(argv[2])
        request.initial_pose.position.y = 0. #float(argv[3])
        request.initial_pose.position.z = 0. #float(argv[4])

    node.get_logger().info("Sending service request to `/spawn_entity`")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        print('response: %r' % future.result())
    else:
        raise RuntimeError(
            'exception while calling service: %r' % future.exception())

    node.get_logger().info("Done! Shutting down node.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
