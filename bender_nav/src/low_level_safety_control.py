#!/usr/bin/env python

import rospy
import math
import roslib
roslib.load_manifest("bender_nav")

from threading import Thread, Lock
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class CmdVelSafety(object):
    """Basic Bender safety"""
    def __init__(self):
        
        # ros interface
        self.pub = rospy.Publisher('/bender/nav/safety/low_level/cmd_vel', Twist, queue_size=5)
        self.laser_sub = rospy.Subscriber('/bender/sensors/laser_front/scan', LaserScan, self.laser_input_cb)
        self.odom_sub = rospy.Subscriber('/bender/nav/odom', Odometry, self.odom_input_cb)
        self.vel_sub = rospy.Subscriber('/bender/nav/cmd_vel', Twist, self.vel_output_cb)

        self.max_rad = .7
        self.front_laser_dist = .25
        self.curr_vel = 0
        self.sent_vel = 0
        self.stoping_acc = 0.34
        self.min_dist = 30.0
        self.min_ang = 2 * math.pi

        # clock
        self.rate_pub = rospy.Rate(10)

        # last message
        self.last_msg = Twist()
        self.last_msg_time = rospy.Time.now()

        # thread for publishing stuff
        self.msg_lock = Lock()
        self.thread = Thread(target=self.publish_state)
        self.thread.start()

    def get_correction_factor(self):
        return 1 + (self.curr_vel ** 2 / (2 * self.stoping_acc))

    def laser_input_cb(self, msg):
        self.min_dist = 30.0
        self.min_ang = 2 * math.pi
        with self.msg_lock:
            for i in range(len(msg.ranges)):
                if msg.range_min <= msg.ranges[i] <= msg.range_max and msg.ranges[i] < self.min_dist:
                    self.min_ang = msg.angle_min + i * msg.angle_increment
                    self.min_dist = math.sqrt(self.front_laser_dist ** 2 + msg.ranges[i] ** 2 - 2 * self.front_laser_dist * msg.ranges[i] * math.cos(self.min_ang))
        #rospy.loginfo("Min laser distance: %f m at %f rads" % (self.min_dist, self.min_ang))
        return

    def odom_input_cb(self, msg):
        self.curr_vel = abs(msg.twist.twist.linear.x)

    def vel_output_cb(self, msg):
        self.sent_vel = msg.linear.x

    def publish_state(self):
        try:
            while not rospy.is_shutdown():
                with self.msg_lock:
                    actual_angle = self.min_dist * math.sin(self.min_ang + math.pi / 2) / self.min_dist
                    rospy.loginfo("Closer point at %f rads and %f m from the center" % (actual_angle, self.min_dist))
                    if self.min_dist <= self.max_rad * self.get_correction_factor() and (actual_angle * self.sent_vel < 0) :
                        rospy.loginfo("Collision detected, stopping movement")
                        self.pub.publish(Twist())
                self.rate_pub.sleep()
        except:
            rospy.logerr("Stopping safety controller.")


def main():
    rospy.init_node('cmd_vel_safety', anonymous=True)
    safe = CmdVelSafety()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: 
        pass
