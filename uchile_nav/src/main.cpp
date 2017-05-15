/*
 * main.cpp
 *
 *  Created on: Nov 15, 2013
 *      Author: matias.pavez.b@gmail.com
 */

#include "ros/ros.h"
#include "uchile_nav/GoalServer.h"
#include <boost/scoped_ptr.hpp>

using namespace uchile_nav;

using std::endl;
using std::cout;

int main(int argc, char** argv) {

	ros::init(argc, argv, "goal_server");

	boost::scoped_ptr<GoalServer> server(new GoalServer(ros::this_node::getName()));

	ros::Rate r(20);
	int counter = 0;
	while (ros::ok()) {

		if (counter < 1) {
			server->update();
		}
		counter = (counter+1)%4; // update at // 5 hz

		ros::spinOnce();
		r.sleep();
	}

	cout << "\n\nQuitting ... \n" << endl;

	return 0;
}
