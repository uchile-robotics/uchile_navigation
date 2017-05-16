/*
 * GoalServerState.cpp
 *
 *  Created on: Jan, 2014
 *      Author: matias.pavez.b@gmail.com
 */

#include "uchile_nav/GoalServerState.h"
#include "uchile_nav/GoalServer.h"
#include <uchile_util/ParameterServerWrapper.h>

using namespace uchile_nav;

/* * * * * * * *
 * INIT STATE  *
 * * * * * * * */

InitState::InitState(uchile_nav::GoalServer* goalServer): server(goalServer) {

	name = "InitState";
}

InitState::~InitState() {

	// 'server' must not be deleted!, because he calls this method
	ROS_INFO_STREAM("[" << name << "] deleted");
}

bool InitState::setQuiet() {

	ROS_INFO_STREAM("[" << name << "]: Setting Quiet State");
	server->setState(server->getQuietState());
	server->setArrivedState(GOAL_WAITING);
	return true;
}

// Not allowed transitions
bool InitState::setGoal(geometry_msgs::PoseStamped goal) {

	ROS_ERROR_STREAM("[" << name << "] Bad usage: Cannot set goal while initializing");
	return false;
}

bool InitState::cancelGoal() {

	ROS_ERROR_STREAM("[" << name << "] Bad usage: Cannot cancel an imaginary goal");
	return false;
}

bool InitState::setAlmostReached() {

	ROS_ERROR_STREAM("[" << name << "] Bad usage: Cannot set almost reach goal while standing in place");
	return false;
}

bool InitState::setReached() {

	ROS_ERROR_STREAM("[" << name << "] Bad usage: Cannot reach goal while standing in place");
	return false;
}

void InitState::updateTrackState() {

	// Nothing to update
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

/* * * * * * * *
 * QUIET STATE *
 * * * * * * * */

 QuietState::QuietState(
			uchile_nav::GoalServer* goalServer,
			boost::shared_ptr<GoalHandler> goalHandler
		): server(goalServer), handler(goalHandler) {

	 name = "QuietState";
}

QuietState::~QuietState() {

	// 'server' must not be deleted!, because he calls this method
	ROS_INFO_STREAM("[" << name << "] deleted");
}

bool QuietState::setGoal(geometry_msgs::PoseStamped goal) {

	handler->spreadRobotPose();

	if (handler->sendGoal(goal)) {

		ROS_INFO_STREAM("[" << name << "] Setting Walking State");

		server->setState(server->getWalkingState());
		server->setArrivedState(GOAL_WALKING);

		return true;

	} else {

		ROS_ERROR_STREAM("[" << name << "] There was a problem while setting the goal :s");
		return false;
	}
}

// Not allowed transitions
bool QuietState::setQuiet() {

	ROS_ERROR_STREAM("[" << name << "] Bad usage: Already quiet");
	return false;
}

bool QuietState::cancelGoal() {

	ROS_ERROR_STREAM("[" << name << "] Bad usage: Cannot cancel an imaginary goal");
	return false;
}

bool QuietState::setAlmostReached() {

	ROS_ERROR_STREAM("[" << name << "] Bad usage: Cannot almost reach goal while standing in place");
	return false;
}

bool QuietState::setReached() {

	ROS_ERROR_STREAM("[" << name << "] Bad usage: Cannot reach goal while standing in place");
	return false;
}

void QuietState::updateTrackState() {

	// Nothing to update
	//ROS_WARN("update quiet");
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -


/* * * * * * * * *
 * WALKING STATE *
 * * * * * * * * */

 WalkingState::WalkingState(
			uchile_nav::GoalServer* goalServer,
			boost::shared_ptr<GoalHandler> goalHandler
		): server(goalServer), handler(goalHandler){

	name = "WalkingState";

	uchile_util::ParameterServerWrapper psw;
	psw.getParameter("goal_almost_reach_radius",_goal_almost_reach_radius,0.3);
}

WalkingState::~WalkingState() {

	// 'server' must not be deleted!, because he calls this method
	ROS_INFO_STREAM("[" << name << "] deleted");
}

bool WalkingState::setGoal(geometry_msgs::PoseStamped goal) {

	ROS_INFO_STREAM("[" << name << "] Resetting goal");

	handler->spreadRobotPose();
	if (handler->sendGoal(goal)) {

		return true;

	} else {

		ROS_ERROR_STREAM("There was a problem while setting the goal :s");

		server->setState(server->getQuietState());
		server->setArrivedState(GOAL_ABORTED);
		handler->cancelGoal();
		return false;
	}
}

bool WalkingState::cancelGoal() {

	ROS_INFO_STREAM("[" << name << "] Canceling current goal, setting quiet state");
	server->setState(server->getQuietState());
	server->setArrivedState(GOAL_CANCELED);

	return handler->cancelGoal();
}

bool WalkingState::setAlmostReached() {

	ROS_INFO_STREAM("[" << name << "] Setting Almost reached state");
	server->setState(server->getAlmostReachedState());
	server->setArrivedState(GOAL_ALMOST_REACHED);

	return true;
}

// Not allowed transitions

bool WalkingState::setQuiet() {

	ROS_ERROR_STREAM("[" << name << "] Bad usage: Cannot set quiet while pursuing a goal");
	return false;
}

bool WalkingState::setReached() {

	ROS_ERROR_STREAM("[" << name << "] Bad usage: Cannot reach goal if i'm not in Almost Reached state");
	return false;
}

void WalkingState::updateTrackState() {

	if( handler->distanceToGoal() < _goal_almost_reach_radius ) {

		this->setAlmostReached();
	}
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -


/* * * * * * * * * * * *
 * ALMOST REACH STATE  *
 * * * * * * * * * * * */

 AlmostReachState::AlmostReachState(
			uchile_nav::GoalServer* goalServer,
			boost::shared_ptr<GoalHandler> goalHandler
		): server(goalServer), handler(goalHandler) {

	uchile_util::ParameterServerWrapper psw;
    psw.getParameter("map_frame",_map_frame,"/map");
    psw.getParameter("goal_reach_degree_th",_goal_reach_degree_th,20);

	name = "AlmostReachState";
}

AlmostReachState::~AlmostReachState() {

	// 'server' must not be deleted!, because he calls this method
	ROS_INFO_STREAM("[" << name << "] deleted");
}

bool AlmostReachState::setGoal(geometry_msgs::PoseStamped goal) {

	ROS_INFO_STREAM("[" << name << "] Resetting goal");
	handler->spreadRobotPose();

	if (handler->sendGoal(goal)) {

		server->setState(server->getWalkingState());
		server->setArrivedState(GOAL_WALKING);

		return true;

	} else {

		ROS_ERROR_STREAM("There was a problem while setting the goal :s");
		server->setState(server->getQuietState());
		server->setArrivedState(GOAL_ABORTED);
		handler->cancelGoal();
		return false;
	}
}

bool AlmostReachState::cancelGoal() {

	ROS_INFO_STREAM("[" << name << "] Canceling goal, setting quiet state");
	server->setState(server->getQuietState());
	server->setArrivedState(GOAL_CANCELED);
	return handler->cancelGoal();
}

bool AlmostReachState::setReached() {

	ROS_INFO_STREAM("[" << name << "] Goal Reached, setting quiet state");
	server->setState(server->getQuietState());
	server->setArrivedState(GOAL_REACHED);

	return true;
}

// Not allowed transitions
bool AlmostReachState::setQuiet() {

	ROS_ERROR_STREAM("[" << name << "] Bad usage: Cannot set quiet while pursuing a goal");
	return false;
}

bool AlmostReachState::setAlmostReached() {

	ROS_ERROR_STREAM("[" << name << "] Bad usage: Already in Almost Reach state");
	return false;
}

void AlmostReachState::updateTrackState() {

	if ( handler->isGoalReady() ||
		fabs( handler->degreesToGoal() ) < _goal_reach_degree_th ) {

		// Goal Reached
		handler->cancelGoal();
		ROS_INFO("Angular adjustment done");
		this->setReached();

	} else {

		// Adjust orientation
		ROS_INFO("Orientation correction in progress");

		geometry_msgs::PoseStamped goal;
		goal.header.frame_id = _map_frame;
		goal.header.stamp = ros::Time::now();
		goal.pose.orientation = handler->getCurrentGoal().pose.orientation;
		goal.pose.position = handler->getCurrentPose().pose.position;

		handler->sendGoal(goal);
	}
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
