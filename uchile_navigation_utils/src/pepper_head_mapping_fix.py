#!/usr/bin/env python  
import qi
import sys
import time
import numpy as np
import rospy
from uchile_util.ros import benpy

from move_base_msgs.msg import MoveBaseActionFeedback
from actionlib_msgs.msg import GoalStatusArray
from uchile_srvs.srv import Onoff
from geometry_msgs.msg import Twist

class Choripan(object):
	def __init__(self):
		ip = "127.0.0.1"
		port = "9559"
		self.session = qi.Session()

		try:
			self.session.connect("tcp://" + str(ip) + ":" + str(port))
		except RuntimeError:
			print ("Connection Error!")
			sys.exit(1)

		self.motion =  self.session.service("ALMotion")

		rospy.init_node('head_fix')
		self.random_sub = rospy.Subscriber('/cmd_vel', Twist, self.main, queue_size=1)
		self.count = 0.




	def main(self,msg):	
		self.motion.setStiffnesses("Head",1.0)
		if self.count == 0:
			self.motion.setAngles("Head",[0.5,0.4],0.05)
		if self.count == 1:
			self.motion.setAngles("Head",[0.0,0.4],0.05)
		if self.count == 2:
			self.motion.setAngles("Head",[-0.5,0.4],0.05)
		if self.count == 3:
			self.motion.setAngles("Head",[0.0,0.4],0.05)
			self.count = -1
		#print motion.getAngles("Head",False)ros
		rospy.sleep(2)
		self.count += 1
		return



if __name__=="__main__":

	pepper = Choripan()

	rospy.spin()

