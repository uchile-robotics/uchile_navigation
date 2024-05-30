#!/usr/bin/env python
# fish
#import roslib; roslib.load_manifest('smach_tutorials')
import rospy
# para la maquina de estado
import smach
import smach_ros
import actionlib
import roslib
# mover la base
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# para enviar en angulos
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped, Twist

class Move():
    def __init__(self):
        self.pose = self

    def set_pose(self,x,y,theta):
        self.pose.x = x
        self.pose.y = y
        self.pose.theta = theta

    def get_pose(self):
        print(self.pose.x, self.pose.y, self.pose.theta)

    def euler_to_cuater(self):
        q = tf.transformations.quaternion_from_euler(0, 0, self.pose.theta/180.0*math.pi, 'rxyz')
        return Quaternion(q[0], q[1], q[2], q[3])

    def movebase_client(self):

        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

        client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = self.pose.x #2
        goal.target_pose.pose.position.y = self.pose.y #3

        goal.target_pose.pose.orientation = self.euler_to_cuater()

        client.send_goal(goal)

        wait = client.wait_for_result()

        if not wait:
            # fail
            rospy.logerr("server not available!")
            rospy.signal_shutdown("server not available!")
        else:
            # go
            return client.get_result()

    def go(self):
        try:
            result = self.movebase_client()
            if result:
                rospy.loginfo("Goal execution done!")
        except rospy.ROSInterruptException:
            rospy.loginfo("Navigation test finished.")

################################################################################


