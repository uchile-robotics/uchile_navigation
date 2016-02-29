/*
 * GoalCalculator.h
 *
 *  Created on: Dec 13, 2013
 *      Author: matias.pavez.b@gmail.com
 */

#ifndef GOALCALCULATOR_H_
#define GOALCALCULATOR_H_

// C, C++
#include <queue>
#include <cmath>
#include <boost/shared_ptr.hpp>

// ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>

// Messages
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/GridCells.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Path.h>

// Parameter Server Wrapper
#include <bender_utils/ParameterServerWrapper.h>

typedef std::vector<geometry_msgs::Point>::iterator pointArray_it;
typedef std::vector<geometry_msgs::Point32> array_32;
typedef std::vector<sensor_msgs::ChannelFloat32> channel_32;
typedef std::vector<float> float_array;

namespace bender_nav {

class GoalCalculator {

private:

	std::string _name;

	// - - - - - Parameters - - - - - - -
	bool _is_map_static;
	std::string _map_frame;
	std::string _obstacles_topic;
	std::string _map_service;
	double _tf_buffer_size; // seconds
	double _approach_delta_degree;
	double _approach_delta_radius;  // [m]
	double _approach_max_radius;    // [m]
	double _approach_person_radius; // [m]
	double _approach_obstacle_th;   // [m] max length from point to obstacle
	double _approach_window_w;      // [m]
	double _approach_window_h;      // [m]
	int	_approach_unknown_th;       // # max unknown


	// - - - - - Variables - - - - - - - -
	nav_msgs::OccupancyGrid _map;
	nav_msgs::GridCells obstacles;
	ros::Subscriber globalObstacles_sub;
	boost::shared_ptr<tf::TransformListener> _tf_listener;

	// Publishers
	ros::Publisher polygon_pub;
	ros::Publisher points_pub;

	// Clients
	ros::ServiceClient _make_plan_client;
	ros::ServiceClient _get_map_client;

public:
	GoalCalculator(std::string name);
	virtual ~GoalCalculator();

	geometry_msgs::PoseStamped calculeLookGoal(geometry_msgs::PoseStamped current_pose, geometry_msgs::PoseStamped look_pose);
	geometry_msgs::PoseStamped calculeRotationGoal(geometry_msgs::PoseStamped current_pose, double degrees);
	geometry_msgs::PoseStamped calculeApproachGoal(geometry_msgs::PoseStamped current_pose, geometry_msgs::PoseStamped approach_pose);

	bool transformPose(geometry_msgs::PoseStamped goal, geometry_msgs::PoseStamped &transformed_pose);

	void callback_globalObstacles(const nav_msgs::GridCells msg);
	void publishRectangle(geometry_msgs::Point* point, double W, double H);

private:
	bool hasFreeSpace(nav_msgs::GridCells obstacles, geometry_msgs::Point point);
};

} /* namespace bender_nav */
#endif /* GOALCALCULATOR_H_ */
