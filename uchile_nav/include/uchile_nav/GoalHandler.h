/*
 * GoalServerAux.h
 *
 *  Created on: Nov 20, 2013
 *      Author: matias.pavez.b@gmail.com
 */

#ifndef GOALSERVERAUX_H_
#define GOALSERVERAUX_H_

// C, C++
#include <iostream>
#include <string>
#include <queue>
#include <cmath>
#include <boost/shared_ptr.hpp>
#include <uchile_nav/GoalStates.h>

// ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>

// Messages
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>

// Move Base Client
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// Parameter Server Wrapper
#include <bender_utils/ParameterServerWrapper.h>

namespace uchile_nav {

class GoalHandler {

private:

	std::string _aux_name;

	geometry_msgs::PoseStamped _current_goal;
	geometry_msgs::PoseStamped _current_pose;

	// - - - - - Parameters - - - - - - -
	std::string _map_frame;
	std::string _pose_topic;
	std::string _input_initial_pose_topic;
	std::string _output_initial_pose_topic;
	std::string _base_frame;
	double _tf_buffer_size; // seconds
	float _goal_sigma_x;
	float _goal_sigma_y;
	float _goal_sigma_angle;

	// publishers
	ros::Publisher _initial_pose_pub;

	// Listeners
	boost::shared_ptr<tf::TransformListener> _tf_listener;
	ros::Subscriber _estimated_pose_sub;
	ros::Subscriber _initial_pose_sub;

	// Clients
	boost::shared_ptr<MoveBaseClient> _mbc;

	// done callback stuff
	bool _is_goal_done;
	void _goalDoneCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr &result) {
		ROS_INFO_STREAM("Goal finished by move_base simple action client");
		_is_goal_done = true;
	}


public:
	GoalHandler(std::string name);
	virtual ~GoalHandler();

	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	// - - - - - - - S u b s c r i b e r   C a l l b a c k s  -  - - - - - - - - - -
	void callback_currentPose(const geometry_msgs::PoseWithCovarianceStamped pose);
	void callback_initialPose(const geometry_msgs::PoseWithCovarianceStamped pose);

	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	// - - - - - -  G o a l   H a n d l i n g   M e t h o d s  - - - - - - - - - - -
	bool spreadRobotPose();
	bool sendGoal(geometry_msgs::PoseStamped goal);
	bool cancelGoal();
	float distanceToGoal();
	float degreesToGoal();
	state_t checkAbortedState();
	bool isGoalReady() {
		return _is_goal_done;
	}

	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	// - - - - - - - - - - - A c c e s s o r s   - - - - - - - - - - - - - - - - - -
	geometry_msgs::PoseStamped getCurrentGoal();
	geometry_msgs::PoseStamped getCurrentPose();
};

} /* namespace uchile_nav */
#endif /* GOALSERVERAUX_H_ */
