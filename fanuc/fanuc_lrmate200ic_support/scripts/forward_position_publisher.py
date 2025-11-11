#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class JointPositionPublisher(Node):
    def __init__(self):
        super().__init__('joint_position_publisher')

        self.publisher_ = self.create_publisher(Float64MultiArray,
                                                '/forward_position_controller/commands',
                                                10)

        # Timer to publish every 0.5 seconds
        self.timer = self.create_timer(0.5, self.timer_callback)

        # Example joint target positions (radians)
        self.joint_targets = [-0.5, 0.3, 0.4, 0.5, 0.2, 0.0]

        self.get_logger().info("Joint Position Publisher started.")

    def timer_callback(self):
        msg = Float64MultiArray()
        msg.data = self.joint_targets
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing joint targets: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = JointPositionPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
