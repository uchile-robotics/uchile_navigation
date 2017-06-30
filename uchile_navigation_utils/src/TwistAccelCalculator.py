#!/usr/bin/env python
# by matias.pavez.b@gmail.com

import roslib
roslib.load_manifest('uchile_nav')
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
from termcolor import colored


class TwistAccelerationPublisher():

    def __init__(self):

        self.max_time_span = 1  # 1[s]

        # time
        self.t0 = None
        self.times = np.array([])
        self.timelag = 0

        # values
        self.vels_a = np.array([])
        self.vels_x = np.array([])

        # subscriber
        self.twist_sub = rospy.Subscriber('/bender/nav/odom', Odometry, self.twist_callback)

        # publisher
        self.vels_pub = rospy.Publisher('~vels', Twist, queue_size=10)
        self.accel_pub = rospy.Publisher('~accels', Twist, queue_size=10)

        rospy.loginfo('ready to work . . .')

    def clear_data(self):
        self.times = np.array([])
        self.vels_a = np.array([])
        self.vels_x = np.array([])

    def centered_diff(self, times, vels):
        dt = np.sum(np.diff(times))
        return (vels[2] - vels[0]) / dt

    def twist_callback(self, msg):

        if self.t0 is None:
            # compute timelag the first time only
            print(' - recording initial time stamp')
            self.t0 = msg.header.stamp.to_sec()
            self.timelag = max(0.0, rospy.get_time() - self.t0)

        elif msg.header.stamp.to_sec() <= self.t0:
            # reset timelag if necessary
            print(' - loop in time (are you using a rosbag?). resetting time.')
            self.timelag = 0
            self.clear_data()

        # data
        twist = msg.twist.twist
        vel_linear = twist.linear.x
        vel_angular = twist.angular.z
        t_msg_real = msg.header.stamp.to_sec()
        t_msg = t_msg_real - self.t0

        # clear old data
        t_now_real = rospy.get_time()
        t_now = t_now_real - self.t0 - self.timelag
        if t_now - t_msg > self.max_time_span:
            print(' - Data is too old.  clearing buffer...')

            # DEBUG PRINTS
            # print('    - max delay       : %s' % self.max_time_span)
            # print('    - lag             : %s' % self.timelag)
            # print('    - init time       : %s' % self.t0)
            # print('    - now  time (real): %s' % t_now_real)
            # print('    - msg  time (real): %s' % t_msg_real)
            # print('    - now  time       : %s' % t_now)
            # print('    - msg  time       : %s' % t_msg)
            self.clear_data()

        # append new values
        self.times = np.append(self.times, np.array([t_msg]))
        self.vels_x = np.append(self.vels_x, np.array([vel_linear]))
        self.vels_a = np.append(self.vels_a, np.array([vel_angular]))
        data_len = len(self.times)

        # we will not compute derivatives using two few datapoints
        #print str(data_len)
        if data_len < 3:
            return

        # delete the oldest element
        if data_len == 4:
            self.times = self.times[1:4]
            self.vels_x = self.vels_x[1:4]
            self.vels_a = self.vels_a[1:4]

        # compute accels
        vel_msg = Twist()
        acc_msg = Twist()
        vel_msg.linear.x = vel_linear
        vel_msg.angular.z = vel_angular
        acc_msg.linear.x = self.centered_diff(self.times, self.vels_x)
        acc_msg.angular.z = self.centered_diff(self.times, self.vels_a)
        self.vels_pub.publish(vel_msg)
        self.accel_pub.publish(acc_msg)

        vx_str = colored('%5.2f' % vel_linear       , 'magenta', attrs=['bold'])
        ax_str = colored('%5.2f' % acc_msg.linear.x , 'magenta', attrs=['bold'])
        va_str = colored('%5.2f' % vel_angular      , 'magenta', attrs=['bold'])
        aa_str = colored('%5.2f' % acc_msg.angular.z, 'magenta', attrs=['bold'])
        print('vx: %s   ax: %s   va: %s   aa: %s' % (vx_str, ax_str, va_str, aa_str))


def main():

    rospy.init_node('twist_accel_publisher')

    TwistAccelerationPublisher()
    rospy.spin()

if __name__ == '__main__':
    main()
