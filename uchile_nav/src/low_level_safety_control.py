#!/usr/bin/env python

import rospy
import math
import numpy
import roslib
import tf
roslib.load_manifest("uchile_nav")

from threading import Thread, Lock
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from uchile_srvs.srv import Transformer


class CmdVelSafety(object):
    """Basic Bender safety"""
    def __init__(self):
        self.laser_front_param = rospy.get_param("bender_sensors_laser_front_link",
                                                 "/bender/sensors/laser_front_link")
        self.laser_rear_param = rospy.get_param("bender_sensors_laser_rear_link",
                                                "/bender/sensors/laser_rear_link")
        self.bender_base_frame = rospy.get_param("base_link", "/bender/base_link")
        self.listener = tf.TransformListener()

        self.tf_client = rospy.ServiceProxy("/bender/tf/simple_pose_transformer/transform", Transformer)
        self.laser_front_closest_point = [float("inf"), math.pi]
        self.laser_rear_closest_point = [float("inf"), math.pi]

        laser_pose = self.get_laser_front_base_transform(0, 0).pose_out
        self.laser_front_base_dist = self._distance([
                                            laser_pose.pose.position.x, 
                                            laser_pose.pose.position.y,
                                            laser_pose.pose.position.z], 
                                            [0,0,0])

        laser_pose = self.get_laser_rear_base_transform(0, 0).pose_out
        self.laser_rear_base_dist = self._distance([
                                            laser_pose.pose.position.x,
                                            laser_pose.pose.position.y, 
                                            laser_pose.pose.position.z], 
                                            [0,0,0])


        # ros interface
        self.pub = rospy.Publisher('/bender/nav/safety/low_level/cmd_vel', Twist, queue_size=5)
        self.laser_front_sub = rospy.Subscriber('/bender/sensors/laser_front/scan', LaserScan, self.laser_front_input_cb)
        self.laser_rear_sub = rospy.Subscriber('/bender/sensors/laser_rear/scan', LaserScan, self.laser_rear_input_cb)
        self.vel_sub = rospy.Subscriber("/bender/nav/low_level_mux/cmd_vel", Twist, self.vel_output_cb)

        self.max_rad = .6
        self.laser_range = math.pi / 6
        self.front_laser_dist = .25
        self.curr_vel = 0
        self.sent_vel = 0
        self.stoping_acc = 0.34

        # clock
        self.rate_pub = rospy.Rate(10)

        # last message
        self.last_msg = Twist()
        self.last_msg_time = rospy.Time.now()

        # thread for publishing stuff
        self.msg_lock = Lock()
        self.msg_lock_2 = Lock()
        self.thread = Thread(target=self.publish_state)
        self.thread.start()

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
        dist = math.sqrt((pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2 + (pos1[2] - pos2[2]) ** 2)
        return dist

    def get_correction_factor(self, obj_rotation):
        vel_factor = 1 + self.sent_vel ** 2 / (2 * self.stoping_acc)
        ang_factor = math.cos(obj_rotation)
        return vel_factor * ang_factor

    def get_laser_front_base_transform(self, dist, ang):
        pose = PoseStamped()
        pose.header.frame_id = self.laser_front_param

        pose.pose.position.x = dist * math.cos(ang)
        pose.pose.position.y = dist * math.sin(ang)
        
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

        pose.pose.position.x = dist * math.cos(ang)
        pose.pose.position.y = dist * math.sin(ang)
        
        orien = tf.transformations.quaternion_from_euler(0, 0, ang)
        pose.pose.orientation.z = orien[2]
        pose.pose.orientation.w = orien[3]
        
        transformer = Transformer()
        transformer.pose_in = pose
        transformer.frame_out = self.bender_base_frame
        
        out = self.tf_client(transformer.pose_in, transformer.frame_out)
        return out

    def laser_front_input_cb(self, msg):
        min_dist = float("inf")
        min_ang = math.pi
        curr_values = [0, msg.ranges[0], msg.ranges[1]]
        with self.msg_lock:
            for i in range(2, len(msg.ranges)):
                curr_values[0] = curr_values[1]
                curr_values[1] = curr_values[2]
                curr_values[2] = msg.ranges[i]

                curr_mean = numpy.mean(curr_values)
                #curr_std = numpy.std(curr_values)

                #if curr_std < curr_mean / 10:
                if msg.range_min <= curr_mean <= msg.range_max:
                    curr_ang = msg.angle_min + i * msg.angle_increment
                    base_ang = math.atan2(math.sin(curr_ang) * curr_mean, self.laser_front_base_dist + math.cos(curr_ang) * curr_mean)
                    if abs(base_ang) < self.laser_range:
                        curr_dist = math.sqrt(self.laser_front_base_dist ** 2 + curr_mean ** 2 + 2 * self.laser_front_base_dist * curr_mean * math.cos(curr_ang))
                        if curr_dist < min_dist:
                            min_ang = base_ang
                            min_dist = curr_dist
            self.laser_front_closest_point = [min_dist, min_ang]

    def laser_rear_input_cb(self, msg):
        min_dist = float("inf")
        min_ang = math.pi
        curr_values = [0, msg.ranges[0], msg.ranges[1]]
        with self.msg_lock:
            for i in range(2, len(msg.ranges)):
                curr_values[0] = curr_values[1]
                curr_values[1] = curr_values[2]
                curr_values[2] = msg.ranges[i]

                curr_mean = numpy.mean(curr_values)
             #   curr_std = numpy.std(curr_values)

              #  if curr_std < curr_mean / 10:
                if msg.range_min <= curr_mean <= msg.range_max:
                    curr_ang = msg.angle_min + i * msg.angle_increment
                    base_ang = math.atan2(math.sin(curr_ang) * curr_mean, self.laser_rear_base_dist + math.cos(curr_ang) * curr_mean)
                    if abs(base_ang) < self.laser_range:
                        curr_dist = math.sqrt(self.laser_rear_base_dist ** 2 + curr_mean ** 2 + 2 * self.laser_rear_base_dist * curr_mean * math.cos(curr_ang))
                        if curr_dist < min_dist:
                            min_ang = base_ang
                            min_dist = curr_dist
            self.laser_rear_closest_point = [min_dist, min_ang]

    def odom_input_cb(self, msg):
        self.curr_vel = abs(msg.twist.twist.linear.x)

    def vel_output_cb(self, msg):
        self.sent_vel = msg.linear.x

    def publish_state(self):
        try:
            while not rospy.is_shutdown():
                with self.msg_lock and self.msg_lock_2:

                    trans_front = self.laser_front_closest_point[0]
                    rot_front = self.laser_front_closest_point[1]

                    trans_rear = self.laser_rear_closest_point[0]
                    rot_rear = math.pi + self.laser_rear_closest_point[1]

                    dist_front = self._distance([
                                                trans_front * math.cos(rot_front), 
                                                trans_front * math.sin(rot_front), 
                                                0], 
                                                [0,0,0])
                    dist_rear  = self._distance([
                                                trans_rear * math.cos(rot_rear), 
                                                trans_rear * math.sin(rot_rear), 
                                                0], 
                                                [0,0,0])
                    #rospy.loginfo("dist front %f m, dist rear %f m" % (dist_front, dist_rear))
                    closest = min(dist_rear, dist_front)
                    clos_ang = rot_front if dist_front < dist_rear else rot_rear
                    corr_factor = self.get_correction_factor(clos_ang)
                    rospy.loginfo("Closer point at %f m from the center with a %f correction_factor" % (closest, corr_factor))
                    if closest <= self.max_rad * abs(corr_factor) and corr_factor * self.sent_vel > 0:
                        rospy.loginfo("Collision detected, stopping movement")
                        self.pub.publish(Twist())
                self.rate_pub.sleep()
        except Exception as e:
            rospy.logerr("Stopping safety controller. Because %s" % e)


def main():
    rospy.init_node('cmd_vel_low_level_safety', anonymous=True)
    safe = CmdVelSafety()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
