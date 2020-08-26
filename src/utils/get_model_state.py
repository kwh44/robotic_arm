import rclpy
from geometry_msgs.msg import Point
import rclpy.qos as qos



def object_type_1_callback(m):
    print(m)

rclpy.init(args=None)
object_type_1_node = rclpy.create_node("mara_object_type_1_node")
qos = qos.QoSProfile(history=1, depth=1)
sub_object_type_1 = object_type_1_node.create_subscription(Point, '/spawned_object/coke_can_entity_state', object_type_1_callback,
                                                                     qos_profile=qos)
rclpy.spin(object_type_1_node)
rclpy.shutdown()



