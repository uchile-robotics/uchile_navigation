
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.action import Spin
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from uchile_navigation.exceptions import NotInitializedError
import numpy as np
import time

class NavigationSkill(Node):
    def __init__(self):
        super().__init__('navigation_skill')
        self._nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._spin_client = ActionClient(self, Spin, 'spin')
        self._goal_handle = None
        self._pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self._pose_callback,
            10
        )
        self.current_pose = None
        self._get_result_future = None
        self._nav_goal_handle = None
        self._spin_goal_handle = None
        self._nav_result_future = None
        self._spin_result_future = None
        self.last_result = None

        if not self.check_server_init():
            raise NotInitializedError("Server is not available, remember to launch the robot's launch files")
    
    def check_server_init(self):
        spin_ok = self._spin_client.wait_for_server(timeout_sec=5.0)  # <-- NUEVO
        nav_ok = self._nav_to_pose_client.wait_for_server(timeout_sec=5.0) 
        return nav_ok and spin_ok

    def _pose_callback(self, msg):
        self.current_pose = msg.pose.pose 
    
    def _parse_frame_id(self, use_robot_frame=False):
        return 'base_link' if use_robot_frame else 'map'
        
    def where_am_i(self):
        """ Returns x, y and theta position in the map reference frame
        """
        q = self.current_pose.orientation
        theta = euler_from_quaternion([q.x, q.y, q.z, q.w])
        x = self.current_pose.position.x
        y = self.current_pose.position.y
        return (x, y, theta)

    def is_localized(self) -> bool:
        return self.current_pose is not None


    def go_to_point(self, x, y, theta=0.0, use_robot_frame=False):
        """ Sets a goal for the robot in a given frame.
        if using robot frame, makes the robot as the origin of the reference frame
        if it is 'map' it will navigate according to the origin of the global reference frame
        In other words, given a map, sets the goal for the robot
        to a specific point in that map.

        Remember that the ROS convention for coordinates are 
        positive x axis forward
        positive y axis left
        theta increases counter clockwise and its value varies between -pi and pi.
        if interested, please see https://www.ros.org/reps/rep-0103.html
        """
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = self._parse_frame_id(use_robot_frame)
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y

        q = quaternion_from_euler(0, 0, theta)
        goal_msg.pose.pose.orientation.x = q[0]
        goal_msg.pose.pose.orientation.y = q[1]
        goal_msg.pose.pose.orientation.z = q[2]
        goal_msg.pose.pose.orientation.w = q[3]

        self._nav_result_future = self._nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )
        self._nav_result_future.add_done_callback(self._goal_response_callback)


    def _feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback}')

    def _goal_response_callback(self, future):
        self._nav_goal_handle = future.result()
        if not self._nav_goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        self.get_logger().info('Goal accepted')
        result_future = self._nav_goal_handle.get_result_async()
        result_future.add_done_callback(self._reached_callback)

    def cancel(self) -> bool:
        """ Cancels the current goal
        Returns True if cancelled correctly
        Returns False if there is no goal to cancel
        """
        if self._goal_handle:
            self.get_logger().info('Cancelling current goal...')
            cancel_future = self._goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self._cancel_callback)
            return True
        self.get_logger().warn('No active goal to cancel.')
        return False

    def spin_in_place(self, angle_radians: float) -> bool:
        """Command the robot to spin in place by a given angle (in radians)."""
        goal_msg = Spin.Goal()
        goal_msg.target_yaw = angle_radians
        goal_msg.time_allowance = rclpy.duration.Duration(seconds=15.0).to_msg()

        self.get_logger().info(f"Sending spin goal: {angle_radians:.2f} rad")
        self._spin_result_future = self._spin_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback_spin
        )
        self._spin_result_future.add_done_callback(self._goal_response_callback_spin)

    def _feedback_callback_spin(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Spin feedback: {feedback}")

    def _goal_response_callback_spin(self, future):
        self._spin_goal_handle = future.result()
        if not self._spin_goal_handle.accepted:
            self.get_logger().info('Spin goal rejected')
            return
        self.get_logger().info('Spin goal accepted')
        result_future = self._spin_goal_handle.get_result_async()
        result_future.add_done_callback(self._reached_callback)

    def wait_for_result(self, timeout=None, action_type="navigate"):
        """Waits for the result of the last sent goal.
        action_type: "navigate" or "spin"
        """
        if action_type == "navigate":
            future = self._nav_result_future
        elif action_type == "spin":
            future = self._spin_result_future
        else:
            self.get_logger().error("Unknown action type.")
            return None

        if future is not None:
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
            if future.done():
                return self.reached()
            else:
                self.get_logger().warn('Timed out waiting for result.')
                return None
        else:
            self.get_logger().warn('No goal has been sent yet.')
            return None

    def _cancel_callback(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully cancelled.')
        else:
            self.get_logger().info('Failed to cancel goal.')


    def _reached_callback(self, future):
        result = future.result()
        self.last_result = result.result
        status_code = result.status

        if status_code == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation succeeded')
        elif status_code == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('Navigation was canceled')
        elif status_code == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn('Navigation aborted')
        else:
            self.get_logger().warn(f'Navigation ended with status code: {status_code}')

    def reached(self) -> bool:
        """Returns true if the last goal succeeded."""
        if self._get_result_future is None:
            return False
        result = self._get_result_future.result()
        return result.status == GoalStatus.STATUS_SUCCEEDED

    def look_point(self, x: float, y: float) -> None:
        """ Makes the robot look to a given point in space
        """    
        theta = np.arctan2(y, x)
        self.go_to_point(0.0, 0.0, theta, use_robot_frame=False)

def main(args=None):
    rclpy.init(args=args)
    nav_skill = NavigationSkill()
    time.sleep(2)  
    nav_skill.go_to_point(1.0, 2.0, 0.0)

    rclpy.spin(nav_skill)
    nav_skill.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
