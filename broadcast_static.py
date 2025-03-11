import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf_transformations import quaternion_from_euler


class StaticFramePublisher(Node):
    """
    Broadcast transforms that never change.

    This example publishes transforms from `world` to a static turtle frame.
    The transforms are only published once at startup, and are constant for all
    time.
    """

    def __init__(self):
        # Inherit
        super().__init__('static_turtle_tf2_broadcaster')
        # Create tf broadcaster
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        # Create tf message
        t = TransformStamped()
        # Fill tf message
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'dummy_turtle'
        t.transform.translation.x = 1.
        t.transform.translation.y = 2.
        t.transform.translation.z = 3.
        quat = quaternion_from_euler(0., 0., -math.pi/4)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        # Broadcast tf message
        self.tf_static_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    static_tf_node = StaticFramePublisher()
    rclpy.spin(static_tf_node)
    static_tf_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()