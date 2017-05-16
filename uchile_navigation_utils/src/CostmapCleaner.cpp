
#include "ros/ros.h"
#include <std_srvs/Empty.h>
#include <uchile_util/ParameterServerWrapper.h>

int main(int argc, char **argv) {

	ros::init(argc, argv, "costmap_cleaner");
	ros::NodeHandle priv("~");

	float dt; // [s]
	uchile_util::ParameterServerWrapper psw;
	psw.getParameter("dt", dt, 5);

	ros::ServiceClient clear_client = priv.serviceClient<std_srvs::Empty>("/bender/nav/move_base/clear_costmaps");
	while ( ros::ok() && !clear_client.waitForExistence(ros::Duration(3.0)) ) ;

	ROS_INFO("Ready to clean ...");

	ros::Rate r(1.0/dt);
	std_srvs::Empty srv;
	while(ros::ok())
	{
		try {
			clear_client.call(srv);
		} catch (std::exception &e) {
			ROS_WARN_STREAM(e.what());
		}
		r.sleep();
	}
	printf("\nQuitting... \n\n");
	return 0;
}
