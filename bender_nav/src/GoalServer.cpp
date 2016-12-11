/*
 * GoalServer.cpp
 *
 *  Created on: Nov 15, 2013
 *      Author: matias.pavez.b@gmail.com
 */

#include "bender_nav/GoalServer.h"

//	TODO: GoalServer Documentation

namespace bender_nav {

GoalServer::GoalServer(std::string name): server_name(name) {

    ros::NodeHandle priv("~");

    // - - - - - - - P A R A M E T E R   S E R V E R - - - - - - - - -
    bender_utils::ParameterServerWrapper psw;
    psw.getParameter("goal_topic",_goal_topic,"goal");


	// - - - - - - - A u x i l i a r y    G u y s - - - - - - - - - - - -
    _goal_handler = boost::shared_ptr<GoalHandler>(new GoalHandler(server_name));
    _goal_calculator = boost::shared_ptr<GoalCalculator>(new GoalCalculator(server_name));


    // - - - - - - - - - - - - - - S T A T E S - - - - - - - - - - - - - -
    _init_state =  boost::shared_ptr<GoalServerState>(new InitState(this));
    _quiet_state = boost::shared_ptr<GoalServerState>(new QuietState(this,_goal_handler));
    _walking_state = boost::shared_ptr<GoalServerState>(new WalkingState(this,_goal_handler));
    _almost_reached_state = boost::shared_ptr<GoalServerState>(new AlmostReachState(this,_goal_handler));
    _arrivedState = GOAL_WAITING;
    this->setState(_init_state);


    // - - - - - - - - - - - - - - S E R V I C E S - - - - - - - - - - - - - -
	_go_to_pose_srv = priv.advertiseService("go", &GoalServer::goToPose,this);
	_look_to_pose_srv  = priv.advertiseService("look", &GoalServer::lookToPose,this);
	_approach_to_pose_srv = priv.advertiseService("approach", &GoalServer::approachToPose,this);
	_cancel_goal_srv = priv.advertiseService("cancel", &GoalServer::cancelGoal,this);
    _has_arrived_srv = priv.advertiseService("has_arrived", &GoalServer::hasArrived,this);
    _get_current_pose_srv = priv.advertiseService("get_current_pose", &GoalServer::getCurrentPose,this);


    // - - - - - - - - - - - - - S U B S C R I B E R S - - - - - - - - - - - - - - -
	_new_goal_sub  = priv.subscribe(_goal_topic, 1, &GoalServer::callback_newGoal,this);

    // We are done with the initialization stuff
    ROS_INFO("[%s] goal server is working ... ", server_name.data());
	_server_state->setQuiet();
}

GoalServer::~GoalServer() {

	ROS_INFO("[%s] goal server was deleted ... ", server_name.data());
}

void GoalServer::update() {

	_server_state->updateTrackState();
}

int GoalServer::getArrivedState() {

	bender_nav::state_t action_state = _goal_handler->checkAbortedState();

	if (action_state == GOAL_ABORTED) {
		return action_state;
	}

	return this->_arrivedState;
}

void GoalServer::setArrivedState(int state) {

	this->_arrivedState = state;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// - - - - - - - - - - - - - - - - - - - - -  S e r v i c e s  - - - - - - - - - - - - - - - - - - - - - - - - - -

bool GoalServer::goToPose(bender_srvs::NavGoal::Request &req, bender_srvs::NavGoal::Response &res) {

	geometry_msgs::PoseStamped transformed_goal;

	if (!_goal_calculator->transformPose(req.goal, transformed_goal)) {
		return false;
	}

	return _server_state->setGoal(transformed_goal);
}

bool GoalServer::lookToPose(bender_srvs::NavGoal::Request &req, bender_srvs::NavGoal::Response &res) {

	geometry_msgs::PoseStamped transformed_goal;
	geometry_msgs::PoseStamped goal;

	if (!_goal_calculator->transformPose(req.goal, transformed_goal) ) {
		return false;
	}
	goal = _goal_calculator->calculeLookGoal(_goal_handler->getCurrentPose(),transformed_goal);

	return _server_state->setGoal(goal);
}

bool GoalServer::approachToPose(bender_srvs::NavGoal::Request &req, bender_srvs::NavGoal::Response &res) {

	geometry_msgs::PoseStamped transformed_goal;
	geometry_msgs::PoseStamped goal;

	// first: cancel the current goal, in order to be able to generate plans
	_server_state->cancelGoal();

	// transform goal position to the map frame
	if (!_goal_calculator->transformPose(req.goal, transformed_goal) ) {
		return false;
	}

	// compute a valid goal
	goal = _goal_calculator->calculeApproachGoal(_goal_handler->getCurrentPose(), transformed_goal);

	return _server_state->setGoal(goal);
}

bool GoalServer::cancelGoal(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {

	return _server_state->cancelGoal();
}

bool GoalServer::hasArrived(bender_srvs::NavGoal::Request  &req, bender_srvs::NavGoal::Response &res){

	res.state = this->getArrivedState();
    return true;
}

bool GoalServer::getCurrentPose(bender_srvs::PoseStamped::Request &req, bender_srvs::PoseStamped::Response &res){

	res.pose_out = _goal_handler->getCurrentPose();
	return true;
}


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// - - - - - - - - - - - S u b s c r i b e r   C a l l b a c k s  -  - - - - - - - - - -

void GoalServer::callback_newGoal(const geometry_msgs::PoseStamped new_goal) {

	geometry_msgs::PoseStamped transformed_goal;

	if (_goal_calculator->transformPose(new_goal, transformed_goal)) {
		
		_server_state->setGoal(transformed_goal);
	}
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// - - - - - -  S t a t e    t r a n s i t i o n    m e t h o d s  - - - - - - - - - - - - - - - - - - - - - - - -

void GoalServer::setState(boost::shared_ptr<GoalServerState> state) {
	this->_server_state.swap(state);
}

boost::shared_ptr<GoalServerState> GoalServer::getInitState() {
	return this->_init_state;
}

boost::shared_ptr<GoalServerState> GoalServer::getQuietState() {
	return this->_quiet_state;
}

boost::shared_ptr<GoalServerState> GoalServer::getWalkingState() {
	return this->_walking_state;
}

boost::shared_ptr<GoalServerState> GoalServer::getAlmostReachedState() {
	return this->_almost_reached_state;
}
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -


} /* namespace bender_nav */
