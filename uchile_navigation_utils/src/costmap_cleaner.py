#!/usr/bin/env python
import rospy
from std_srvs.srv import Empty


if __name__ == "__main__":
	rospy.init_node('costmap_cleaner')
	service_name = rospy.get_param("service_name", "/maqui/nav/move_base/clear_costmaps")   
	dt = rospy.get_param('dt', 5.0)
	rospy.loginfo("Waiting for service: {}".format(service_name))
	rospy.wait_for_service(service_name)
	print "Service {} available".format(service_name)
	service_client = rospy.ServiceProxy(service_name,Empty)

	rate = rospy.Rate(1.0/dt)
	while(not rospy.is_shutdown()):
		try:
			service_client()
			rospy.loginfo("Calling service: {}".format(service_name))
		except rospy.ServiceException, e:
			rospy.logwarn("Service call failed: {}".format(e))

		rate.sleep()
	rospy.loginfo("Exit...")
	