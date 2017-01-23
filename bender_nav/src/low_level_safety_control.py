#!/usr/bin/env python

import rospy
import math
import roslib
import tf
roslib.load_manifest("bender_nav")

from threading import Thread, Lock
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class CmdVelSafety(object):
    """Basic Bender safety"""
    def __init__(self):
        self.laser_front_param = rospy.get_param("bender_sensors_laser_front_link",
                                                 "/bender/sensors/laser_front_link")
        self.laser_rear_param = rospy.get_param("bender_sensors_laser_rear_link",
            "/bender/sensors/laser_rear_link")
        self.bender_base_frame = rospy.get_param("base_link", "/bender/base_link")
        self.listener = tf.TransformListener()

        # ros interface
        self.pub = rospy.Publisher('/bender/nav/safety/low_level/cmd_vel', Twist, queue_size=5)
        self.laser_sub = rospy.Subscriber('/bender/sensors/laser_front/scan', LaserScan, self.laser_front_input_cb)
        self.odom_sub = rospy.Subscriber('/bender/nav/odom', Odometry, self.odom_input_cb)
        self.vel_sub = rospy.Subscriber('/bender/nav/cmd_vel', Twist, self.vel_output_cb)

        self.max_rad = .6
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
        vel_factor = 1 + max((self.curr_vel ** 2 / (2 * self.stoping_acc)), (self.sent_vel ** 2 / (2 * self.stoping_acc)))
        ang_factor = 1 - 2 * obj_rotation[2] ** 2
        return vel_factor * ang_factor

    def laser_front_input_cb(self, msg):
        min_dist = float("inf")
        min_ang = math.pi
        with self.msg_lock:
            for i in range(len(msg.ranges)):
                if msg.range_min <= msg.ranges[i] <= msg.range_max and msg.ranges[i] < min_dist:
                    min_ang = msg.angle_min + i * msg.angle_increment
                    min_dist = msg.ranges[i]

        br = tf.TransformBroadcaster()
        br.sendTransform((min_dist * math.cos(min_ang), min_dist * math.sin(min_ang), 0),
                         tf.transformations.quaternion_from_euler(0, 0, min_ang),
                         rospy.Time.now(),
                         "/closest_front_laser_point",
                         self.laser_front_param)
        #rospy.loginfo("Min laser distance: %f m x %f m y (%f m total), at %f rads" % (min_dist * math.cos(min_ang), min_dist * math.sin(min_ang), min_dist, min_ang))

    def laser_rear_input_cb(self, msg):
        min_dist = math.inf
        with self.msg_lock:
            for i in range(len(msg.ranges)):
                if msg.range_min <= msg.ranges[i] <= msg.range_max and msg.ranges[i] < min_dist:
                    min_ang = msg.angle_min + i * msg.angle_increment
                    min_dist = msg.ranges[i]

        br = tf.TransformBroadcaster()
        br.sendTransform((min_dist * math.cos(min_ang), min_dist * math.sin(min_ang), 0),
            tf.transformations.quaternion_from_euler(0, 0, -min_ang),
            rospy.Time.now(),
            "/closest_rear_laser_point",
            self.laser_front_param)
        #rospy.loginfo("Min laser distance: %f m at %f rads" % (min_dist, self.min_ang))
        return

    def odom_input_cb(self, msg):
        self.curr_vel = abs(msg.twist.twist.linear.x)

    def vel_output_cb(self, msg):
        self.sent_vel = msg.linear.x

    def publish_state(self):
        self.listener.waitForTransform("/closest_front_laser_point",
            self.bender_base_frame, rospy.Time(), rospy.Duration(1.0))
        #self.listener.waitForTransform("/closest_rear_laser_point",
        #    self.bender_base_frame, rospy.Time(), rospy.Duration(1.0))
        
        try:
            while not rospy.is_shutdown():
                with self.msg_lock:
                    now = rospy.Time.now()
                    # Closer point in the front
                    self.listener.waitForTransform("/closest_front_laser_point",
                        self.bender_base_frame, rospy.Time(), rospy.Duration(2.0))
                    (trans_front, rot_front) = self.listener.lookupTransform("/closest_front_laser_point",
                        self.bender_base_frame, rospy.Time())

                    # Closer point in the back
#                    listener.waitForTransformFull("/closest_rear_laser_point", now,
#                        self.bender_base_frame, rospy.Duration(1.0))
#                    (trans_rear, rot_rear) = listener.lookupTransformFull("/closest_rear_laser_point", now,
#                        self.bender_base_frame)
#
#                    closest = min(self._distance(trans_front, [0,0,0]), self._distance(trans_rear, [0,0,0]))
                    closest = self._distance(trans_front, [0,0,0])
                    corr_factor = self.get_correction_factor(rot_front)
                    rospy.loginfo("Closer point at %f m from the center with a %f correction_factor and %f m/s sent velocity" % (closest, corr_factor, self.sent_vel))
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
