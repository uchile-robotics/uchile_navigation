#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
import numpy as np
import heapq
import random
import tf2_ros
import rclpy.time

from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA


class AutoExplorerFFP(Node):
    def __init__(self):
        super().__init__('auto_explorer_ffp')

        self.subscription = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.frontier_marker_pub = self.create_publisher(Marker, '/frontier_markers', 1)

        self.map_info = None
        self.last_goal = None
        self.last_candidates = []
        self.failed_goals = set()
        self.arrival_threshold = 0.40
        self.wait_count = 0
        self.wait_limit = 5
        self.no_frontier_count = 0  # contador de ciclos sin fronteras

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def get_robot_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            return x, y
        except Exception as e:
            self.get_logger().warn(f"No se pudo obtener la transformación: {e}")
            return None, None

    def map_callback(self, msg):
        self.map_info = msg.info
        map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))

        robot_x, robot_y = self.get_robot_pose()
        if robot_x is None or robot_y is None:
            self.get_logger().warn("No robot pose available from TF. Skipping this cycle.")
            return

        if self.last_goal:
            dx = robot_x - self.last_goal[0]
            dy = robot_y - self.last_goal[1]
            dist_to_goal = np.hypot(dx, dy)
            if dist_to_goal > self.arrival_threshold:
                self.wait_count += 1
                self.get_logger().info(
                    f"Robot still heading to goal ({dist_to_goal:.2f} m away). Attempt {self.wait_count}/{self.wait_limit}")
                if self.wait_count < self.wait_limit:
                    return
                else:
                    self.get_logger().warn("Exceeded wait limit. Marking goal as failed.")
                    self.failed_goals.add(self.last_goal)
                    self.wait_count = 0
            else:
                self.get_logger().info("Robot reached the goal.")
                self.wait_count = 0

        frontiers = self.fast_front_propagation(map_data, robot_x, robot_y)
        self.get_logger().info(f"Detected {len(frontiers)} frontier candidates.")
        self.publish_frontier_markers(frontiers)

        filtered = []
        for fx, fy in frontiers:
            if self.obstacle_filter(map_data, fx, fy) and self.boundary_filter(map_data, fx, fy):
                filtered.append((fx, fy))

        # --- Finalización por ciclos sin fronteras válidas ---
        if not filtered:
            self.no_frontier_count += 1
            self.get_logger().warn(f"No valid frontiers detected. Count = {self.no_frontier_count}/5")

            if self.no_frontier_count >= 5:
                self.get_logger().info("Exploración finalizada: 5 ciclos sin fronteras válidas.")
                rclpy.shutdown()
                return

            # Fallbacks normales
            if self.last_candidates:
                for _, wx, wy in self.last_candidates[1:]:
                    if (wx, wy) in self.failed_goals:
                        continue
                    dx = robot_x - wx
                    dy = robot_y - wy
                    if np.hypot(dx, dy) >= 0.9:
                        self.last_goal = (wx, wy)
                        self.publish_goal(wx, wy)
                        return
                self.get_logger().info("No viable backup goal found.")

            rand = self.find_random_explored_cell(map_data)
            if rand:
                wx, wy = rand
                self.last_goal = (wx, wy)
                self.get_logger().warn(f"Sending to random free cell: ({wx:.2f}, {wy:.2f})")
                self.publish_goal(wx, wy)
            else:
                self.get_logger().error("No fallback goal or free space found.")
            return

        heap = []
        for fx, fy in filtered:
            wx, wy = self.grid_to_world(fx, fy)
            dist = np.hypot(wx - robot_x, wy - robot_y)
            if dist >= 0.9 and (wx, wy) not in self.failed_goals:
                heapq.heappush(heap, (dist, wx, wy))

        if heap:
            self.last_candidates = sorted(heap)
            _, wx, wy = self.last_candidates[0]
            self.last_goal = (wx, wy)
            self.publish_goal(wx, wy)
            self.no_frontier_count = 0
        else:
            self.no_frontier_count += 1
            self.get_logger().info("All filtered frontiers too close or previously failed.")
            self.get_logger().info(f"Frontier fail count: {self.no_frontier_count}/5")

            if self.no_frontier_count >= 5:
                self.get_logger().info("Exploración finalizada: 5 ciclos sin fronteras utilizables.")
                rclpy.shutdown()
                return

    def fast_front_propagation(self, map_data, robot_x, robot_y):
        height, width = map_data.shape
        visited = np.zeros((height, width), dtype=bool)
        frontiers = []

        res = self.map_info.resolution
        origin = self.map_info.origin.position
        rx = int((robot_x - origin.x) / res)
        ry = int((robot_y - origin.y) / res)

        if not (0 <= rx < width and 0 <= ry < height):
            self.get_logger().warn("Robot position out of map bounds.")
            return []

        queue = [(rx, ry)]
        visited[ry, rx] = True

        while queue:
            x, y = queue.pop(0)
            if map_data[y, x] != 0:
                continue
            neighbors = self.get_neighbors(x, y, width, height)
            if any(map_data[ny, nx] == -1 for nx, ny in neighbors):
                frontiers.append((x, y))
            for nx, ny in neighbors:
                if not visited[ny, nx] and map_data[ny, nx] == 0:
                    visited[ny, nx] = True
                    queue.append((nx, ny))
        return frontiers

    def publish_frontier_markers(self, frontiers):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "frontiers"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.color = ColorRGBA(r=0.5, g=0.0, b=0.5, a=1.0)
        marker.pose.orientation.w = 1.0

        for fx, fy in frontiers:
            wx, wy = self.grid_to_world(fx, fy)
            p = Point()
            p.x = wx
            p.y = wy
            p.z = 0.05
            marker.points.append(p)

        self.frontier_marker_pub.publish(marker)

    def find_random_explored_cell(self, map_data):
        free = np.argwhere(map_data == 0)
        if len(free) == 0:
            return None
        choice = random.choice(free)
        return self.grid_to_world(choice[1], choice[0])

    def get_neighbors(self, x, y, width, height):
        offsets = [(-1,0), (1,0), (0,-1), (0,1)]
        return [(x+dx, y+dy) for dx, dy in offsets if 0 <= x+dx < width and 0 <= y+dy < height]

    def obstacle_filter(self, map_data, x, y, size=3):
        h, w = map_data.shape
        xmin = max(0, x - size // 2)
        xmax = min(w, x + size // 2 + 1)
        ymin = max(0, y - size // 2)
        ymax = min(h, y + size // 2 + 1)
        patch = map_data[ymin:ymax, xmin:xmax]
        return np.mean(patch == 100) < 0.2

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
        return round(wx, 1), round(wy, 1)

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
    node = AutoExplorerFFP()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

