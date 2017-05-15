/*
 * GoalServer.h
 *
 *  Created on: Nov 15, 2013
 *      Author: matias.pavez.b@gmail.com
 */

#ifndef GOALSERVER_H_
#define GOALSERVER_H_

// C, C++
#include <iostream>
#include <string>
#include <boost/shared_ptr.hpp>

// ROS
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <bender_srvs/NavGoal.h>
#include <bender_srvs/PoseStamped.h>

// Goal Server States
#include "GoalServerState.h"
#include <uchile_nav/GoalStates.h>

// Goal Server Handler object
#include "GoalHandler.h"

// Goal Server auxiliary objects
#include "GoalCalculator.h"
#include <bender_utils/ParameterServerWrapper.h>

namespace uchile_nav {

class GoalServer {

public:

	std::string server_name;

    // - - - - - Server Auxiliary Guys - - - - -
	boost::shared_ptr<GoalHandler> _goal_handler;
	boost::shared_ptr<GoalCalculator> _goal_calculator;

private:

	// - - - - - Parameters - - - - - - -
	std::string _goal_topic;
	int _arrivedState;

	// - - - - - - Services - - - - - -
	ros::ServiceServer _go_to_pose_srv;
	ros::ServiceServer _look_to_pose_srv;
	ros::ServiceServer _approach_to_pose_srv;
	ros::ServiceServer _cancel_goal_srv;
    ros::ServiceServer _has_arrived_srv;
    ros::ServiceServer _get_current_pose_srv;

    // Listeners
    ros::Subscriber _new_goal_sub;

	//  - - - - - Server States - - - - - -
	boost::shared_ptr<GoalServerState> _init_state;
	boost::shared_ptr<GoalServerState> _quiet_state;
	boost::shared_ptr<GoalServerState> _walking_state;
	boost::shared_ptr<GoalServerState> _almost_reached_state;
	boost::shared_ptr<GoalServerState> _server_state;


public:
	GoalServer(std::string name);
	virtual ~GoalServer();
	void update();

	int getArrivedState();
	void setArrivedState(int state);

	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	// - - - - - - - - - - - - - - - - - - - - -  S e r v i c e s  - - - - - - - - - - - - - - - - - - - - - - - - - -

	bool goToPose(bender_srvs::NavGoal::Request &req, bender_srvs::NavGoal::Response &res);

	bool lookToPose(bender_srvs::NavGoal::Request &req, bender_srvs::NavGoal::Response &res);

	bool approachToPose(bender_srvs::NavGoal::Request &req, bender_srvs::NavGoal::Response &res);

	bool cancelGoal(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

	bool hasArrived(bender_srvs::NavGoal::Request &req, bender_srvs::NavGoal::Response &res);

	bool getCurrentPose(bender_srvs::PoseStamped::Request &req, bender_srvs::PoseStamped::Response &res);

	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	// - - - - - - - S u b s c r i b e r   C a l l b a c k s  -  - - - - - - - - - -
	void callback_newGoal(const geometry_msgs::PoseStamped new_goal);

	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	// - - - - - -  S t a t e    t r a n s i t i o n    m e t h o d s  - - - - - - - - - - - - - - - - - - - - - - - -

	void setState(boost::shared_ptr<GoalServerState> state);

	boost::shared_ptr<GoalServerState> getInitState();

	boost::shared_ptr<GoalServerState> getQuietState();

	boost::shared_ptr<GoalServerState> getWalkingState();

	boost::shared_ptr<GoalServerState> getAlmostReachedState();

	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

};

} /* namespace uchile_nav */
#endif /* GOALSERVER_H_ */
