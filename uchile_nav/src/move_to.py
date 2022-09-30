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

class Foo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state FOO')
        if self.counter < 2:
            self.counter += 1
            rospy.init_node('fish')
            m = Move()
            m.set_pose(5, 0, 60)
            #m.get_pose()
            m.go()
            return 'outcome1'
        else:
            return 'outcome2'
            rospy.init_node('fish')
            m = Move()
            m.set_pose(2, 3, 150)
            #m.get_pose()
            m.go()

class Bar(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])

    def execute(self, userdata):
        rospy.loginfo('Executing state BAR')
        rospy.init_node('fish')
        m = Move()
        m.set_pose(6.5, 5, 180)
        #m.get_pose()
        m.go()
        return 'outcome1'

def main():
    rospy.init_node('fish')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('FOO', Foo(),
                               transitions={'outcome1':'BAR', 'outcome2':'outcome4'})
        smach.StateMachine.add('BAR', Bar(),
                               transitions={'outcome1':'FOO'})

    # Execute SMACH plan
    outcome = sm.execute()



if __name__ == '__main__':
    main()
'''
if __name__ == '__main__':
    rospy.init_node('fish')
    m = Move()
    m.set_pose(1, 3, 60)
    #m.get_pose()
    m.go()
'''
