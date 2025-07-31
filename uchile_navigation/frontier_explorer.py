#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
import numpy as np


class AutoExplorer(Node):
    def __init__(self):
        super().__init__('auto_explorer')

        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)

        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)

        self.map_info = None
        self.current_pose = None


    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose


    def map_callback(self, msg):
        self.map_info = msg.info
        map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))

        frontiers = self.detect_frontiers_ffp(map_data)
        self.get_logger().info(f"Detected {len(frontiers)} raw frontier candidates")

        # Filtrar fronteras espurias
        valid_frontiers = []
        for fx, fy in frontiers:
            if self.obstacle_filter(map_data, fx, fy) and self.boundary_filter(map_data, fx, fy):
                valid_frontiers.append((fx, fy))

        if not valid_frontiers or self.current_pose is None:
            self.get_logger().info("No valid frontiers or pose not available.")
            return

        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y

        # Ordenar por distancia desde el robot
        candidates = sorted(
            [(self.grid_to_world(fx, fy), fx, fy) for fx, fy in valid_frontiers],
            key=lambda item: np.hypot(item[0][0] - robot_x, item[0][1] - robot_y)
        )

        for (wx, wy), gx, gy in candidates:
            dist = np.hypot(wx - robot_x, wy - robot_y)
            if dist >= 0.4:
                self.publish_goal(wx, wy)
                return
        self.get_logger().info("All valid frontiers are too close.")


    def detect_frontiers_ffp(self, map_data):
        height, width = map_data.shape
        known = np.zeros_like(map_data, dtype=bool)
        front = []

        queue = [(0, 0)]
        while queue:
            x, y = queue.pop(0)
            if not (0 <= x < width and 0 <= y < height):
                continue
            if known[y, x]:
                continue
            known[y, x] = True

            if map_data[y, x] == -1:
                neighbors = self.get_neighbors(x, y, width, height)
                for nx, ny in neighbors:
                    if map_data[ny, nx] == 0:
                        front.append((x, y))
                        break
                queue.extend(neighbors)
        return front


    def get_neighbors(self, x, y, width, height):
        offsets = [(-1,0),(1,0),(0,-1),(0,1)]
        return [(x+dx, y+dy) for dx, dy in offsets if 0 <= x+dx < width and 0 <= y+dy < height]


    def obstacle_filter(self, map_data, x, y, size=3):
        h, w = map_data.shape
        xmin = max(0, x - size // 2)
        xmax = min(w, x + size // 2 + 1)
        ymin = max(0, y - size // 2)
        ymax = min(h, y + size // 2 + 1)
        patch = map_data[ymin:ymax, xmin:xmax]
        return np.mean(patch == 100) < 0.2  # menos de 20% obstáculos


    def boundary_filter(self, map_data, x, y, size=3):
        h, w = map_data.shape
        xmin = max(0, x - size // 2)
        xmax = min(w, x + size // 2 + 1)
        ymin = max(0, y - size // 2)
        ymax = min(h, y + size // 2 + 1)
        patch = map_data[ymin:ymax, xmin:xmax]
        total = patch.size
        unknown = np.sum(patch == -1)
        rho = 1 - 2 * abs((unknown / total) - 0.5)
        return rho > 0.5


    def grid_to_world(self, x, y):
        res = self.map_info.resolution
        origin = self.map_info.origin.position
        wx = origin.x + (x + 0.5) * res
        wy = origin.y + (y + 0.5) * res
        return wx, wy


    def publish_goal(self, wx, wy):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = wx
        goal.pose.position.y = wy
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0
        self.goal_publisher.publish(goal)
        self.get_logger().info(f"Published goal: ({wx:.2f}, {wy:.2f})")


def main(args=None):
    rclpy.init(args=args)
    node = AutoExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()