# taken from https://zmk5.github.io/general/demo/2019/07/15/ros2-spawning-entity.html

import os
import sys
import rclpy
from multiprocessing import Process
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity

def spawn_object(model):
    rclpy.init()
    node = rclpy.create_node("entity_spawner")
    
    node.get_logger().info(
        'Creating Service client to connect to `/spawn_entity`')
    client = node.create_client(SpawnEntity, "/spawn_entity")

    node.get_logger().info("Connecting to `/spawn_entity` service...")
    if not client.service_is_ready():
        client.wait_for_service()
        node.get_logger().info("...connected!")

    request = SpawnEntity.Request()
    request.name = model[0]
    request.xml = open(model[1], 'r').read()
    request.robot_namespace = "spawned_object"
    request.initial_pose.position.x, request.initial_pose.position.y, request.initial_pose.position.z = model[2]
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


def main():

    beer = ["beer", "/root/robotic_arm/models/beer/model.sdf", (10., 10., 0.)]
    coke = ["coke_can", "/root/robotic_arm/models/coke_can/model.sdf", (5., 5., 0.)]
    mara = ["mara", "/root/robotic_arm/models/mara/model.sdf", (0., 0., 0.)]

    processes = []
    
    processes.append(Process(target=spawn_object, args=(beer, )))
    processes.append(Process(target=spawn_object, args=(coke, )))
    processes.append(Process(target=spawn_object, args=(mara, )))

    for process in processes:
        process.start()

    for process in processes:
        process.join()


if __name__ == '__main__':
    main()
