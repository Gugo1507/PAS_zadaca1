#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory


class TrajectoryCommander(Node):
    def __init__(self):
        super().__init__('trajectory_commander')

        self._client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )

        self.joint_names = [
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4",
            "joint_5",
            "joint_6",
        ]

        self.get_logger().info("Waiting for trajectory action server...")
        self._client.wait_for_server()
        self.get_logger().info("Trajectory action server available!")

        # Send trajectory once
        self.send_trajectory()

    def send_trajectory(self):
        goal_msg = FollowJointTrajectory.Goal()

        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = [0.5, -0.3, 0.4, 0.0, 0.0, 0.0]
        point.time_from_start.sec = 2
        traj.points.append(point)

        goal_msg.trajectory = traj

        self.get_logger().info("Sending trajectory...")
        self._client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryCommander()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
