#!/usr/bin/env python
# by matias.pavez.b@gmail.com

import roslib
roslib.load_manifest('bender_nav')
import rospy
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


import time
import numpy as np
import geometry_msgs


class TwistAccelerationPublisher():

    def __init__(self):

        self.max_time_span = 1 # 1[s]

        # time
        self.t0 = False
        self.times   = np.array([])
        self.timelag = 0

        # values
        self.vels_a = np.array([])
        self.vels_x = np.array([])
        
        # subscriber
        self.twist_sub = rospy.Subscriber('/bender/nav/odom', Odometry, self.twist_callback)
        
        # publisher
        self.vels_pub  = rospy.Publisher('~vels'  , Twist, queue_size=10)
        self.accel_pub = rospy.Publisher('~accels', Twist, queue_size=10)
        
        rospy.loginfo('ready to work . . .')
        
        

    def clear_data(self):
        
        self.times   = np.array([])
        self.vels_a  = np.array([])
        self.vels_x  = np.array([])
        
    def centered_diff(self, times, vels):
        
        dt = np.sum(np.diff(times))
        return (vels[2] - vels[0])/dt
        
        
    def twist_callback(self, msg):

        # compute timelag the first time only
        if not self.t0:
            self.t0 = msg.header.stamp.to_sec()
            self.timelag = rospy.get_time() - self.t0
        
        # reset timelag if necessary
        if msg.header.stamp.to_sec() == self.t0:
            self.timelag = rospy.get_time() - self.t0
            self.clear_data()
            print 'Resetting time'
        
        # data
        twist = msg.twist.twist
        msg_t = msg.header.stamp.to_sec() - self.t0
        vel_linear  = twist.linear.x
        vel_angular = twist.angular.z
        
        # clear old data
        t_now   = rospy.get_time() - self.t0 - self.timelag
        if t_now - msg_t > self.max_time_span:
            self.clear_data()

        # append new values
        self.times = np.append(self.times, np.array([msg_t]))
        self.vels_x = np.append(self.vels_x, np.array([vel_linear ]))
        self.vels_a  = np.append(self.vels_a , np.array([vel_angular]))
        data_len = len(self.times)
        
        # we will not compute derivatives using two few datapoints
        #print str(data_len)
        if data_len < 3:
            return
        
        # delete the oldest element
        if data_len == 4:
            self.times  = self.times [1:4]
            self.vels_x = self.vels_x[1:4]
            self.vels_a = self.vels_a[1:4]
        
        # compute accels
        vel_msg = Twist()
        acc_msg = Twist()
        vel_msg.linear.x  = vel_linear
        vel_msg.angular.z = vel_angular
        acc_msg.linear.x  = self.centered_diff(self.times, self.vels_x)
        acc_msg.angular.z = self.centered_diff(self.times, self.vels_a)
        self.vels_pub.publish(vel_msg)
        self.accel_pub.publish(acc_msg)
        print 'vel=' + str(vel_linear) + ', acc=' + str(acc_msg.linear.x)


def main():

    rospy.init_node('twist_accel_publisher')

    TwistAccelerationPublisher()
    rospy.spin()

if __name__ == '__main__':
    main()
