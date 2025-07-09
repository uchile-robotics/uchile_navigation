
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from uchile_navigation.exceptions import NotInitializedError
import numpy as np
import time

class NavigationSkill(Node):
    def __init__(self):
        super().__init__('navigation_skill')
        self._nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._goal_handle = None
        self._pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self._pose_callback,
            10
        )
        self.current_pose = None
        self._get_result_future = None
        self.last_result = None
        if not self.check_server_init():
            raise NotInitializedError("Server is not available, remember to launch the robot's launch files")
    
    def check_server_init(self):
        return self._nav_to_pose_client.wait_for_server(timeout_sec=5.0)
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

        self._send_goal_future = self._nav_to_pose_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self._goal_response_callback)
        return True

    def _goal_response_callback(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        self.get_logger().info('Goal accepted')
        self._get_result_future = self._goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._reached_callback)

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
    
    def wait_for_result(self, timeout=None):
        """A method that stops the code to continue running until the robot reaches
        its destination

        Args:
            timeout (int, optional): Time waited until the robot reaches its goal. Defaults to None.

        Returns:
            int or none
             
        """
        if self._get_result_future is not None:
            rclpy.spin_until_future_complete(self, self._get_result_future, timeout_sec=timeout)
            if self._get_result_future.done():
                return self.reached()
            else:
                self.get_logger().warn('Timed out waiting for navigation result.')
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
        self.last_result = future.result().result
        if self.last_result == 0:
            self.get_logger().info('Navigation succeeded')
        elif self.last_result == 1:
            self.get_logger().info('Navigation was canceled')
        else:
            self.get_logger().warn(f'Navigation failed with result code: {self.last_result}')

    def reached(self) -> bool:
        """ Returns true if the last goal was successful
        """
        return self.last_result == 0

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
