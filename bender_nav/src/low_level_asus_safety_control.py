#!/usr/bin/env python

import rospy
from math import sin, cos, atan2, pi, sqrt, pow as mpow
import numpy
import roslib
import tf
roslib.load_manifest("bender_nav")

from threading import Thread, Lock
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from bender_srvs.srv import Transformer


class CmdVelSafety(object):
    """Basic Bender safety"""
    def __init__(self):
        self.laser_front_param = rospy.get_param("bender_sensors_laser_front_link",
                                                 "/bender/sensors/laser_front_link")
        self.laser_rear_param = rospy.get_param("bender_sensors_laser_rear_link",
                                                "/bender/sensors/laser_rear_link")
        self.scan_param = rospy.get_param("bender_sensors_rgbd_head_link",
                                          "/bender/sensors/rgbd_head_depth_optical_frame")
        self.bender_base_frame = rospy.get_param("base_link", "/bender/base_link")
        self.listener = tf.TransformListener()

        self.tf_client = rospy.ServiceProxy("/bender/tf/simple_pose_transformer/transform", Transformer)
        self.laser_front_closest_point = [float("inf"), pi]
        self.laser_rear_closest_point = [float("inf"), pi]
        self.scan_closest_point = [float("inf"), pi]

        laser_pose = self.get_laser_front_base_transform(0, 0).pose_out
        self.laser_front_base_dist = self._distance([
                                                    laser_pose.pose.position.x, 
                                                    laser_pose.pose.position.y,
                                                    laser_pose.pose.position.z], 
                                                    [0,0,0])

        laser_pose = self.get_laser_rear_base_transform(0, 0).pose_out
        self.laser_rear_base_dist =  self._distance([
                                                    laser_pose.pose.position.x,
                                                    laser_pose.pose.position.y,
                                                    laser_pose.pose.position.z], 
                                                    [0,0,0])

        scan_pose = self.get_scan_base_transform(0, 0).pose_out
        self.scan_base_dist =  self._distance([
                                                scan_pose.pose.position.x,
                                                scan_pose.pose.position.y,
                                                0], 
                                                [0,0,0])


        self.max_rad = .55
        self.laser_range = pi / 9
        self.front_laser_dist = .25
        self.curr_vel = 0
        self.sent_vel = 0
        self.stoping_acc = 0.3

        # clock
        self.rate_pub = rospy.Rate(10)
        self.laser_front_cb_rate = rospy.Rate(5)
        self.laser_rear_cb_rate = rospy.Rate(5)
        self.scan_cb_rate = rospy.Rate(5)
        self.cnt_front = 0
        self.cnt_rear = 0

        # ros interface
        self.pub = rospy.Publisher('/bender/nav/safety/low_level/cmd_vel', Twist, queue_size=2)
        self.scan_sub = rospy.Subscriber('/bender/sensors/rgbd_head/scan', LaserScan, self.scan_input_cb, queue_size = 1)
        self.laser_front_sub = rospy.Subscriber('/bender/sensors/laser_front/scan', LaserScan, self.laser_front_input_cb, queue_size = 1)
        self.laser_rear_sub = rospy.Subscriber('/bender/sensors/laser_rear/scan', LaserScan, self.laser_rear_input_cb, queue_size = 1)
        self.vel_sub = rospy.Subscriber("/bender/nav/low_level_mux/cmd_vel", Twist, self.vel_output_cb, queue_size = 1)
        self.odom_sub = rospy.Subscriber("/bender/nav/odom", Odometry, self.odom_input_cb, queue_size = 1)

        # last message
        self.last_msg = Twist()
        self.last_msg_time = rospy.Time.now()
        try:
            while not rospy.is_shutdown():
                trans_front = self.laser_front_closest_point[0]
                rot_front = self.laser_front_closest_point[1]

                trans_rear = self.laser_rear_closest_point[0]
                rot_rear = pi + self.laser_rear_closest_point[1]

                trans_scan = self.scan_closest_point[0]
                rot_scan = self.scan_closest_point[1]

                dist_front = self._distance([
                                            trans_front * cos(rot_front), 
                                            trans_front * sin(rot_front), 
                                            0], 
                                            [0,0,0])

                dist_rear  = self._distance([
                                            trans_rear * cos(rot_rear), 
                                            trans_rear * sin(rot_rear), 
                                            0], 
                                            [0,0,0])

                dist_scan = self._distance([
                                            trans_scan * cos(rot_scan),
                                            trans_scan * sin(rot_scan),
                                            0],
                                            [0,0,0])

                rospy.loginfo("dist front %f m, dist rear %f m, dist scan %f m" % (dist_front, dist_rear, dist_scan))
                closest = min(dist_rear, min(dist_front, dist_scan))
                if dist_front < dist_rear and dist_front < dist_scan:
                    clos_ang = rot_front
                elif dist_rear < dist_front and dist_rear < dist_scan:
                    clos_ang = rot_rear
                else:
                    clos_ang = rot_scan
                corr_factor = self.get_correction_factor(clos_ang)
                #rospy.loginfo("Closer point at %f m from the center with a %f correction_factor" % (closest, corr_factor))
                if closest <= self.max_rad + abs(corr_factor) and corr_factor * self.sent_vel > 0:
                    rospy.loginfo("Collision detected, stopping movement")
                    self.pub.publish(Twist())
                self.rate_pub.sleep()
        except Exception as e:
            rospy.logerr("Stopping safety controller. Because %s" % e)

    def _distance(self, pos1, pos2):
        """
        This method calculates the distance between two positions given in an array [x, y, z].

        Args:
            pos1 (list):
                first position
            pos2 (list):
                second position

        Returns:
            float: Distance between the two positions
        """
        dist = sqrt(mpow((pos1[0] - pos2[0]), 2) + mpow((pos1[1] - pos2[1]), 2) + mpow((pos1[2] - pos2[2]), 2))
        return dist

    def get_correction_factor(self, obj_rotation):
        vel_factor = mpow(max(self.curr_vel, self.sent_vel), 2) / (2 * self.stoping_acc)
        ang_factor = 1 if cos(obj_rotation) > 0 else -1
        return vel_factor * ang_factor

    def get_laser_front_base_transform(self, dist, ang):
        pose = PoseStamped()
        pose.header.frame_id = self.laser_front_param

        pose.pose.position.x = dist * cos(ang)
        pose.pose.position.y = dist * sin(ang)
        
        orien = tf.transformations.quaternion_from_euler(0, 0, ang)
        pose.pose.orientation.z = orien[2]
        pose.pose.orientation.w = orien[3]
        
        transformer = Transformer()
        transformer.pose_in = pose
        transformer.frame_out = self.bender_base_frame
        
        out = self.tf_client(transformer.pose_in, transformer.frame_out)
        return out

    def get_laser_rear_base_transform(self, dist, ang):
        pose = PoseStamped()
        pose.header.frame_id = self.laser_rear_param

        pose.pose.position.x = dist * cos(ang)
        pose.pose.position.y = dist * sin(ang)
        
        orien = tf.transformations.quaternion_from_euler(0, 0, ang)
        pose.pose.orientation.z = orien[2]
        pose.pose.orientation.w = orien[3]
        
        transformer = Transformer()
        transformer.pose_in = pose
        transformer.frame_out = self.bender_base_frame
        
        out = self.tf_client(transformer.pose_in, transformer.frame_out)
        return out

    def get_scan_base_transform(self, dist, ang):
        pose = PoseStamped()
        pose.header.frame_id = self.scan_param

        pose.pose.position.x = dist * cos(ang)
        pose.pose.position.y = dist * sin(ang)
        
        orien = tf.transformations.quaternion_from_euler(0, 0, ang)
        pose.pose.orientation.z = orien[2]
        pose.pose.orientation.w = orien[3]
        
        transformer = Transformer()
        transformer.pose_in = pose
        transformer.frame_out = self.bender_base_frame
        
        out = self.tf_client(transformer.pose_in, transformer.frame_out)
        return out

    def scan_input_cb(self, msg):
        min_dist = float("inf")
        min_ang = pi
        curr_values = [0, msg.ranges[0], msg.ranges[1]]
        for i in range(2, len(msg.ranges)):
            curr_values[0] = curr_values[1]
            curr_values[1] = curr_values[2]
            curr_values[2] = msg.ranges[i]

            curr_mean = numpy.mean(curr_values)
            if msg.range_min <= curr_mean and curr_mean <= msg.range_max:
                curr_ang = msg.angle_min + i * msg.angle_increment
                base_ang = atan2(sin(curr_ang) * curr_mean, self.scan_base_dist + cos(curr_ang) * curr_mean)
                if abs(base_ang) < self.laser_range:
                    curr_dist = sqrt(mpow(self.scan_base_dist, 2) + mpow(curr_mean, 2) + 2 * self.scan_base_dist * curr_mean * cos(curr_ang))
                    if curr_dist < min_dist:
                        min_ang = base_ang
                        min_dist = curr_dist
        self.scan_closest_point = [min_dist, min_ang]
        self.scan_cb_rate.sleep()

    def laser_front_input_cb(self, msg):
        min_dist = float("inf")
        min_ang = pi
        curr_values = [0, msg.ranges[0], msg.ranges[1]]
        for i in range(2, len(msg.ranges)):
            curr_values[0] = curr_values[1]
            curr_values[1] = curr_values[2]
            curr_values[2] = msg.ranges[i]

            curr_mean = numpy.mean(curr_values)
            if msg.range_min <= curr_mean and curr_mean <= msg.range_max:
                curr_ang = msg.angle_min + i * msg.angle_increment
                base_ang = atan2(sin(curr_ang) * curr_mean, self.laser_front_base_dist + cos(curr_ang) * curr_mean)
                if abs(base_ang) < self.laser_range:
                    curr_dist = sqrt(mpow(self.laser_front_base_dist, 2) + mpow(curr_mean, 2) + 2 * self.laser_front_base_dist * curr_mean * cos(curr_ang))
                    if curr_dist < min_dist:
                        min_ang = base_ang
                        min_dist = curr_dist
        self.laser_front_closest_point = [min_dist, min_ang]
        self.laser_front_cb_rate.sleep()

    def laser_rear_input_cb(self, msg):
        min_dist = float("inf")
        min_ang = pi
        curr_values = [0, msg.ranges[0], msg.ranges[1]]
        for i in range(2, len(msg.ranges)):
            curr_values[0] = curr_values[1]
            curr_values[1] = curr_values[2]
            curr_values[2] = msg.ranges[i]

            curr_mean = numpy.mean(curr_values)
            if msg.range_min <= curr_mean <= msg.range_max:
                curr_ang = msg.angle_min + i * msg.angle_increment
                base_ang = atan2(sin(curr_ang) * curr_mean, self.laser_rear_base_dist + cos(curr_ang) * curr_mean)
                if abs(base_ang) < self.laser_range:
                    curr_dist = sqrt(mpow(self.laser_rear_base_dist, 2) + mpow(curr_mean, 2) + 2 * self.laser_rear_base_dist * curr_mean * cos(curr_ang))
                    if curr_dist < min_dist:
                        min_ang = base_ang
                        min_dist = curr_dist
        self.laser_rear_closest_point = [min_dist, min_ang]
        self.laser_rear_cb_rate.sleep()

    def odom_input_cb(self, msg):
        self.curr_vel = abs(msg.twist.twist.linear.x)
        self.rate_pub.sleep()

    def vel_output_cb(self, msg):
        self.sent_vel = msg.linear.x
        #self.rate_pub.sleep()

    # def publish_state(self):
        


def main():
    rospy.init_node('cmd_vel_low_level_safety', anonymous=True)
    safe = CmdVelSafety()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
