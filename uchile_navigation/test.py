#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from nav2_msgs.action import NavigateToPose
from tf_transformations import quaternion_from_euler
from action_msgs.msg import GoalStatus


class NavigateToPoseActionClient(Node):

    def __init__(self):
        super().__init__('navigate_to_pose_client')
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    def send_goal_and_wait(self, x, y, theta):
        # Crear mensaje de goal
        goal_msg = NavigateToPose.Goal()
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, theta)

        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # Esperar servidor
        self.get_logger().info("Waiting for action server...")
        self._action_client.wait_for_server()

        # Fase 1: Enviar goal y esperar respuesta de aceptación
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by server")
            return False

        self.get_logger().info("Goal accepted, navigating...")

        # Fase 2: Esperar a que la acción termine
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        result = get_result_future.result()

        # Revisar estado final
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Navigation succeeded!")
            return True
        elif result.status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn("Navigation aborted!")
            return False
        elif result.status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("Navigation canceled!")
            return False
        else:
            self.get_logger().warn(f"Navigation ended with status: {result.status}")
            return False


def main(args=None):
    rclpy.init(args=args)
    action_client = NavigateToPoseActionClient()

    x = 8.23
    y = 2.64
    theta = 0.0

    success = action_client.send_goal_and_wait(x, y, theta)

    if success:
        action_client.get_logger().info("Robot reached the goal successfully.")
    else:
        action_client.get_logger().warn("Robot failed to reach the goal.")

    action_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
