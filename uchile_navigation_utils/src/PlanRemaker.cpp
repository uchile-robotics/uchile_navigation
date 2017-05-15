#include <cstdlib>
#include <stdlib.h>
#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <bender_utils/ParameterServerWrapper.h>

// ROS services / messages
#include <nav_msgs/Path.h>
#include <nav_msgs/GetPlan.h>
#include <bender_srvs/NavGoal.h>


namespace uchile_nav {

class PlanRemaker {

public:

	PlanRemaker();
	virtual ~PlanRemaker();
	void spin();

private:

	geometry_msgs::PoseStamped _current_goal;
	bool _received_first_goal;

	// parameters
	std::string _map_frame;
	std::string _robot_frame;
	float _plan_tolerance;

	// Service Clients
	ros::ServiceClient _get_plan_client;
	ros::ServiceClient _arrived_client;

	// Listeners
	ros::Subscriber _new_goal_sub;
	ros::Subscriber _new_plan_sub;
	tf::TransformListener _tf_listener;

	bool request_plan(nav_msgs::Path& plan, const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal, const float tolerance) {

		if ( !_get_plan_client.waitForExistence(ros::Duration(0.3)) ) {
			ROS_WARN_STREAM("cannot request navigation plan... server unavailable: " << _get_plan_client.getService() );
			return false;
		}

		nav_msgs::GetPlan srv;
		srv.request.start = start;
		srv.request.goal = goal;
		srv.request.tolerance = tolerance;

		try {
			_get_plan_client.call(srv);
			plan = srv.response.plan;

		} catch (std::exception &e) {
			ROS_WARN_STREAM("Failed to get plan from server: " << _get_plan_client.getService());
			return false;
		}

		return true;
	};

	bool getCurrentPoseInMap(geometry_msgs::PoseStamped& robot) {

		geometry_msgs::PoseStamped pose_stamped;
		pose_stamped.header.frame_id = _robot_frame;
		pose_stamped.header.stamp = ros::Time::now();
		pose_stamped.pose.orientation.w = 1;

		// transform pose
		try {
			_tf_listener.waitForTransform(_map_frame, _robot_frame, pose_stamped.header.stamp, ros::Duration(2.0));
			_tf_listener.transformPose(_map_frame, pose_stamped, robot);

		} catch (tf::ConnectivityException &e) {
			ROS_WARN_STREAM("Requested a transform between unconnected trees!: " << e.what());
			return false;

		} catch (tf::TransformException &e) {
			ROS_WARN_STREAM("Transform Exception: " << e.what());
			return false;
		}

		return true;
	}

	bool get_nav_moving_state(bool &is_moving) {

		if ( !_arrived_client.waitForExistence(ros::Duration(0.3)) ) {
			ROS_WARN_STREAM("cannot request navigation state... server unavailable: " << _arrived_client.getService() );
			return false;
		}
		bender_srvs::NavGoal srv;
		try {
			_arrived_client.call(srv);
		} catch (std::exception &e) {
			ROS_WARN_STREAM("Failed to get navigation status from server: " << _arrived_client.getService());
			return false;
		}

		// TODO: cambiar esa definición con números a una distinta!
		is_moving = (srv.response.state == 1);
		return true;
	};

	void callback_newGoal(const geometry_msgs::PoseStamped new_goal) {

		// transform goal
		geometry_msgs::PoseStamped map_goal;
		if (new_goal.header.frame_id != _map_frame) {

			try {
				_tf_listener.waitForTransform(_map_frame, new_goal.header.frame_id, new_goal.header.stamp, ros::Duration(2.0));
				_tf_listener.transformPose(_map_frame, new_goal, map_goal);

			} catch (tf::ConnectivityException &e) {
				ROS_WARN_STREAM("Requested a transform between unconnected trees!: " << e.what());
				return;

			} catch (tf::TransformException &e) {
				ROS_WARN_STREAM("Transform Exception: " << e.what());
				return;
			}

		} else {
			map_goal = new_goal;
		}

		// save goal
		_current_goal = map_goal;
		_received_first_goal = true;
		ROS_INFO_STREAM("Received new goal: (x,y)=(" << std::setprecision(2) << std::fixed
				<< _current_goal.pose.position.x << "," << _current_goal.pose.position.y << ")");
	};

};

PlanRemaker::PlanRemaker() {

	ros::NodeHandle priv("~");
	_received_first_goal = false;

	// - - - - - - - P A R A M E T E R   S E R V E R - - - - - - - - -
	bender_utils::ParameterServerWrapper psw;
	psw.getParameter("map_frame",_map_frame, "/map");
	psw.getParameter("robot_frame",_robot_frame, "/bender/base_link");
	psw.getParameter("plan_tolerance",_plan_tolerance, 0.1);


	// - - - - - - - S U B S C R I B E R S - - - - - - - - - - - - - - -
	// navigation data
	_new_goal_sub  = priv.subscribe("new_goal", 1, &PlanRemaker::callback_newGoal,this);

	// - - - - - - - S E R V I C E   C L I E N T S - - - - - - - - - - - - - -
	_get_plan_client = priv.serviceClient<nav_msgs::GetPlan>("make_plan");
	while ( ros::ok() && !_get_plan_client.waitForExistence(ros::Duration(3.0)) ) ;

	_arrived_client = priv.serviceClient<bender_srvs::NavGoal>("has_arrived");
	while ( ros::ok() && !_arrived_client.waitForExistence(ros::Duration(3.0)) ) ;

	// [important!] wait for timer initialization!
	while (ros::ok() && ros::Time::now().isZero());

}

PlanRemaker::~PlanRemaker() {

}

void PlanRemaker::spin() {

	//ready to work
	if (!_received_first_goal) {
		return;
	}

	bool is_moving;
	if (!get_nav_moving_state(is_moving)) {
		return;
	}

	if (is_moving) {

		ROS_INFO("sending re-planning command");

		// get current position
		geometry_msgs::PoseStamped robot;
		if ( !getCurrentPoseInMap(robot) ) {
			return;
		}

		// generate & execute a new plan
		nav_msgs::Path new_plan;
		request_plan(new_plan, robot, _current_goal, _plan_tolerance);
	}
	else {
		ROS_INFO("currently not re-planning");
	}
}

}


int main(int argc, char **argv){

	ros::init(argc, argv, "plan_remaker");


	boost::scoped_ptr<uchile_nav::PlanRemaker> node(
			new uchile_nav::PlanRemaker()
	);


	float delta_segundos = 5;
	ros::Rate r(1.0/delta_segundos);
	while(ros::ok())
	{
		node->spin();

		r.sleep();
		ros::spinOnce();
	}
	printf("\nQuitting... \n\n");

	return 0;
}
