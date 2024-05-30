#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point
from sound_play.libsoundplay import SoundClient
class map_navigation():

    def choose(self):
        choice='q'
        rospy.loginfo("|-------------------------------|")
        rospy.loginfo("|PRESSE A KEY:")
        rospy.loginfo("|'0': INICIO ")
        rospy.loginfo("|'1': REFRI ")
        rospy.loginfo("|'2': PIEZA")                                       
        rospy.loginfo("|'3': SOFA ")
        rospy.loginfo("|'4': custom")
        rospy.loginfo("|'q': Quit ")
        rospy.loginfo("|-------------------------------|")
        rospy.loginfo("|WHERE TO GO?")
        choice = input()
        return choice

    #def _callback(self, msg):
        #msj = msg.status_list[0]
        #status = msj.status
        #print("----------------------------")
        #print(msj)
        #print( type(msj))
        #print("status:", status)
        #self.status = status

    def __init__(self):

        #self.subscriber = rospy.Subscriber('/move_base/status', GoalStatusArray, self._callback)
        #self.status = 1

        path_to_sounds = "/home/ros/catkin_ws/src/sounds/"
        # declare the coordinates of interest
        self.xinicio = -1.3478
        self.yinicio = 5.6232
        self.xrefri = -1.9517
        self.yrefri = 2.4806
        self.xpieza =  -0.4331
        self.ypieza = 1.4848
        self.xsofa = 1.3478
        self.ysofa = 5.6232
        self.goalReached = False
        # initiliaze
        rospy.init_node('map_navigation', anonymous=False)
        choice = self.choose()

        if (choice == 0):
            self.goalReached = self.moveToGoal(self.xinicio, self.yinicio)
        elif (choice == 1):
            self.goalReached = self.moveToGoal(self.xrefri, self.yrefri)
        elif (choice == 2):
            self.goalReached = self.moveToGoal(self.xsofa, self.ysofa)
        elif (choice == 3):
            self.goalReached = self.moveToGoal(self.xpieza, self.ypieza)
        elif (choice == 4): 
            rospy.loginfo("please enter x:")
            xcustom = input()
            rospy.loginfo("please enter y:")
            ycustom = input()
            rospy.loginfo("please enter w:")
            wcustom = input()
            self.goalReached = self.moveToGoal(xcustom, ycustom)

        if (choice!='q'):
            if (self.goalReached):
                rospy.loginfo("Congratulations!")
        #       rospy.spin()
                #sc.playWave(path_to_sounds+"ship_bell.wav")
            else:
                rospy.loginfo("Hard Luck!")
                #sc.playWave(path_to_sounds+"short_buzzer.wav")

        while choice != 'q':
            choice = self.choose()
            if (choice == 0):
                self.goalReached = self.moveToGoal(self.xinicio, self.yinicio)
            elif (choice == 1):
                self.goalReached = self.moveToGoal(self.xrefri, self.yrefri)
            elif (choice == 2):
                self.goalReached = self.moveToGoal(self.xsofa, self.ysofa)
            elif (choice == 3):
                self.goalReached = self.moveToGoal(self.xpieza, self.ypieza)
            elif (choice == 4): 
                rospy.loginfo("please enter x:")
                xcustom = input()
                rospy.loginfo("please enter y:")
                ycustom = input()
                rospy.loginfo("please enter w:")
                wcustom = input()
                self.goalReached = self.moveToGoal(xcustom, ycustom)

            if (choice!='q'):
                if (self.goalReached):
                    rospy.loginfo("Congratulations!")
                #rospy.spin()
                #sc.playWave(path_to_sounds+"ship_bell.wav")
            else:
                rospy.loginfo("Hard Luck!")
                #sc.playWave(path_to_sounds+"short_buzzer.wav")

    def shutdown(self):
    # stop turtlebot
        rospy.loginfo("Quit program")
        rospy.sleep()

    def moveToGoal(self,xGoal,yGoal):
        #define a client for to send goal requests to the move_base server through a
        #SimpleActionClient
        ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        #wait for the action server to come up
        while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
            rospy.loginfo("Waiting for the move_base action server to come up")

        goal = MoveBaseGoal()
        #set up the frame parameters
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        # moving towards the goal*/
        goal.target_pose.pose.position = Point(xGoal,yGoal,0)
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.0
        goal.target_pose.pose.orientation.w = 1.0
        rospy.loginfo("Sending goal location ...")
        ac.send_goal(goal)

        #while not self.status == 3:
            #print("actual status: ", self.status)

            #if self.status ==4 : 
                #obtener pose actual con amcl_pose
                # dar orden de giro                

        ac.wait_for_result(rospy.Duration(60))

        if(ac.get_state() == GoalStatus.SUCCEEDED):
            rospy.loginfo("You have reached the destination")
            return True
        else:
            rospy.loginfo("The robot failed to reach the destination")
            return False



if __name__ == '__main__':
    try:
        rospy.loginfo("You have reached the destination")
        map_navigation()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("map_navigation node terminated.")