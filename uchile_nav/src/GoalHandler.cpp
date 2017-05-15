/*
 * GoalServerAux.cpp
 *
 *  Created on: Nov 20, 2013
 *      Author: matias.pavez.b@gmail.com
 */

#include "uchile_nav/GoalHandler.h"

using std::string;
using std::exception;

namespace uchile_nav {

GoalHandler::GoalHandler(string name):_aux_name(name) {

    ros::NodeHandle priv("~");

    _is_goal_done = false;

    // - - - - - P A R A M E T E R   S E R V E R - - - - - - - - - - - -
    bender_utils::ParameterServerWrapper psw;
    psw.getParameter("map_frame",_map_frame,"/map");
    psw.getParameter("pose_topic",_pose_topic,"/bender/nav/amcl_pose");
    psw.getParameter("input_initial_pose_topic",_input_initial_pose_topic,"/bender/nav/goal_server/initialpose");
    psw.getParameter("output_initial_pose_topic",_output_initial_pose_topic,"/bender/nav/initialpose");
    psw.getParameter("base_frame",_base_frame,"/bender/base_link");
    psw.getParameter("tf_buffer_size",_tf_buffer_size,3.0);
    psw.getParameter("goal_spread_x",_goal_sigma_x,0.5);
    psw.getParameter("goal_spread_y",_goal_sigma_y,0.5);
    psw.getParameter("goal_spread_theta",_goal_sigma_angle,M_PI/12.0);


    // - - - - - - - - - - - - - - - -  P U B L I S H E R S - - - - - - - - - - - - - - - -
	_initial_pose_pub = priv.advertise<geometry_msgs::PoseWithCovarianceStamped>(_output_initial_pose_topic,1, this);

    // - - - - - - - - - - - - - - - - S U B S C R I B E R S - - - - - - - - - - - - - - -
	_tf_listener = boost::shared_ptr<tf::TransformListener>(
		new tf::TransformListener(ros::Duration(_tf_buffer_size))
	);
	_estimated_pose_sub = priv.subscribe(_pose_topic, 1, &GoalHandler::callback_currentPose,this);
	_initial_pose_sub = priv.subscribe(_input_initial_pose_topic, 1, &GoalHandler::callback_initialPose,this);


	// - - - - - - - - - - - - - - S U B S C R I B E R S - - - - - - - - - - - - - -
	_mbc = boost::shared_ptr<MoveBaseClient>(new MoveBaseClient("move_base",true));
	while(!_mbc->waitForServer(ros::Duration(5.0)) && ros::ok() ) {
		ROS_WARN("Waiting for the move_base action server to come up");
	}

	ROS_INFO_STREAM("[" << _aux_name << "]: handler node created");
}

GoalHandler::~GoalHandler() {

	ROS_INFO_STREAM("[" << _aux_name << "]: auxiliary node deleted");
}


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// - - - - - - - - - - - S u b s c r i b e r   C a l l b a c k s  -  - - - - - - - - - -

void GoalHandler::callback_currentPose(const geometry_msgs::PoseWithCovarianceStamped pose) {

	_current_pose.header = pose.header;
	_current_pose.pose = pose.pose.pose;
}

void GoalHandler::callback_initialPose(const geometry_msgs::PoseWithCovarianceStamped input) {

	// handle frames not starting with '/'. such as the ones from RVIZ
	std::string target_frame = input.header.frame_id;
	if (target_frame.at(0) != '/') {
		target_frame = "/" + target_frame;
	}

	if (target_frame == _map_frame) {
		ROS_INFO_STREAM("[" << _aux_name << "]: Sending initial pose from map frame: " << _map_frame);
		_initial_pose_pub.publish(input);


	} else if (target_frame == _base_frame) {

		geometry_msgs::PoseWithCovarianceStamped output;
		geometry_msgs::PoseStamped tmp_pose, transformed_pose;
		tmp_pose.header = input.header;
		tmp_pose.pose = input.pose.pose;
		try {
			_tf_listener->waitForTransform(_map_frame, _base_frame, ros::Time::now(), ros::Duration(1.0));
			_tf_listener->transformPose(_map_frame, tmp_pose, transformed_pose);

		} catch (tf::TransformException & ex) {
			ROS_ERROR_STREAM("[" << _aux_name << "]: Cannot transform pose because: " << ex.what());			
			return;
		}

		// publish pose
		ROS_INFO_STREAM("[" << _aux_name << "]: Sending initial pose from base frame: " << _base_frame);
		output.header = input.header;
		output.header.frame_id = _map_frame;
		output.pose.covariance = input.pose.covariance;
		output.pose.pose = transformed_pose.pose;
		_initial_pose_pub.publish(output);

	} else {
		ROS_WARN_STREAM("[" << _aux_name << "]: Will not send initial pose for invalid frame '"
			<< target_frame << "'. Valid frames are '" << _map_frame << "' and '" << _base_frame << "'.");
	}
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// - - - - - -  G o a l   H a n d l i n g   M e t h o d s  - - - - - - - - - - - - - - -

bool GoalHandler::spreadRobotPose() {

	geometry_msgs::PoseWithCovarianceStamped ini_pose;

	tf::StampedTransform transform;

	// Initial Pose in map_frame
    ini_pose.header.stamp = ros::Time::now();
	ini_pose.header.frame_id = _map_frame;
	ini_pose.pose.covariance[6*0+0] = _goal_sigma_x * _goal_sigma_x;
	ini_pose.pose.covariance[6*1+1] = _goal_sigma_y * _goal_sigma_y;
	ini_pose.pose.covariance[6*5+5] = _goal_sigma_angle * _goal_sigma_angle;

	try {
		// seek actual pose
		_tf_listener->waitForTransform(_map_frame, _base_frame, ros::Time::now(), ros::Duration(1.0));
		_tf_listener->lookupTransform(_map_frame,_base_frame,ros::Time(0), transform);

	} catch (tf::TransformException& ex) {

		ROS_ERROR("[%s]: Cannot set initial pose because: %s",_aux_name.data(),ex.what());
		return false;

	}

	ini_pose.pose.pose.position.x = transform.getOrigin().x();
	ini_pose.pose.pose.position.y = transform.getOrigin().y();
	ini_pose.pose.pose.position.z = transform.getOrigin().z();
	ini_pose.pose.pose.orientation.x = transform.getRotation().x();
	ini_pose.pose.pose.orientation.y = transform.getRotation().y();
	ini_pose.pose.pose.orientation.z = transform.getRotation().z();
	ini_pose.pose.pose.orientation.w = transform.getRotation().w();

	ROS_INFO("[%s]: Setting initial pose", _aux_name.data());

	// publish pose
	_initial_pose_pub.publish(ini_pose);

	return true;

}

bool GoalHandler::sendGoal(geometry_msgs::PoseStamped goal) {

	move_base_msgs::MoveBaseGoal mb_goal;

	mb_goal.target_pose.header.frame_id = _map_frame;
	mb_goal.target_pose.header.stamp = ros::Time::now();
	mb_goal.target_pose.pose = goal.pose;

	_is_goal_done = false;
	_mbc->sendGoal(mb_goal,
			boost::bind(&GoalHandler::_goalDoneCb, this, _1, _2),
			MoveBaseClient::SimpleActiveCallback(),
			MoveBaseClient::SimpleFeedbackCallback());

	_current_goal = goal;
	ROS_INFO("[%s]: Sending goal %f, %f",_aux_name.data(),goal.pose.position.x,goal.pose.position.y);

	return true;
}

bool GoalHandler::cancelGoal() {

	try {
		_mbc->cancelAllGoals();
		ROS_INFO("goals cancelled");

	} catch (exception& e) {

        ROS_WARN_STREAM("Attempt to cancel all goals, when there is no one!: " <<  e.what());
        return false;
	}

	return true;
}

float GoalHandler::distanceToGoal() {

	float dx, dy;
	_current_pose = getCurrentPose();
	dx = _current_goal.pose.position.x - _current_pose.pose.position.x;
	dy = _current_goal.pose.position.y - _current_pose.pose.position.y;

	return sqrtf(dx*dx + dy*dy);
}

float GoalHandler::degreesToGoal() {

	// Variables útiles
	double roll, pitch, yaw;
	double roll2, pitch2, yaw2;

	tf::Matrix3x3 m1(tf::Quaternion(
		_current_pose.pose.orientation.x,_current_pose.pose.orientation.y,
		_current_pose.pose.orientation.z,_current_pose.pose.orientation.w)
	);

	tf::Matrix3x3 m2(tf::Quaternion(
		_current_goal.pose.orientation.x,_current_goal.pose.orientation.y,
		_current_goal.pose.orientation.z,_current_goal.pose.orientation.w)
	);

	m1.getRPY(roll,pitch,yaw);
	m2.getRPY(roll2,pitch2,yaw2);

	return (yaw - yaw2)*180/M_PI;
}

state_t GoalHandler::checkAbortedState() {

	actionlib::SimpleClientGoalState state = _mbc->getState();

	//ROS_WARN_STREAM("MBC STATE: " << state.toString());

	if (   state == actionlib::SimpleClientGoalState::ABORTED
	    || state == actionlib::SimpleClientGoalState::REJECTED) {

		return uchile_nav::GOAL_ABORTED;

	} else if ( state == actionlib::SimpleClientGoalState::PREEMPTED ) {

		return uchile_nav::GOAL_CANCELED;

	}

	return uchile_nav::GOAL_OTHER;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// - - - - - - - - - - - A c c e s s o r s   - - - - - - - - - - - - - - - - - -

geometry_msgs::PoseStamped GoalHandler::getCurrentGoal() {

	return _current_goal;
}

geometry_msgs::PoseStamped GoalHandler::getCurrentPose() {

	geometry_msgs::PoseStamped ps;
	geometry_msgs::PoseStamped ps_out;

	// Initial Pose in map_frame
	ps.header.frame_id = _base_frame;
	ps.pose.orientation.w = 1;

	bool done = false;
	while (!done && ros::ok()) {
		// transform pose
		try {
			ps.header.stamp = ros::Time::now();
			_tf_listener->waitForTransform(_map_frame, _base_frame, ps.header.stamp, ros::Duration(1.0));
			_tf_listener->transformPose(_map_frame, ps, ps_out);
			done = true;
		} catch (tf::TransformException &e) {
			ROS_WARN_STREAM("Transform Exception: " << e.what());
			done = false;
			ros::Duration(0.1).sleep();
		}
	}
	ps_out.header.stamp = ps.header.stamp;
	ps_out.header.frame_id = _map_frame;
	return ps_out;
}


} /* namespace uchile_nav */
