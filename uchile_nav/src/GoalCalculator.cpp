/*
 * GoalCalculator.cpp
 *
 *  Created on: Dec 13, 2013
 *      Author: matias.pavez.b@gmail.com
 */

#include "uchile_nav/GoalCalculator.h"

using std::string;

namespace uchile_nav {


GoalCalculator::GoalCalculator(string name):_name(name) {

    ros::NodeHandle priv("~");

    // - - - - - -  P A R A M E T E R   S E R V E R - - - - - - - - - - - -
    uchile_util::ParameterServerWrapper psw;
    psw.getParameter("map_frame",_map_frame,"/map");
    psw.getParameter("obstacles_topic",_obstacles_topic, "/bender/nav/move_base/global_costmap/inflated_obstacles");
    psw.getParameter("is_map_static",_is_map_static,true);
    psw.getParameter("map_service",_map_service,"/static_map");
    psw.getParameter("tf_buffer_size", _tf_buffer_size, 3.0);
    psw.getParameter("approach_delta_degree",_approach_delta_degree,50.0);
    psw.getParameter("approach_delta_radius",_approach_delta_radius,0.3);
    psw.getParameter("approach_max_radius",_approach_max_radius,1.0);
    psw.getParameter("approach_person_radius",_approach_person_radius,0.6);
    psw.getParameter("approach_obstacle_th",_approach_obstacle_th,0.1);
    psw.getParameter("approach_window_w",_approach_window_w,0.8);
    psw.getParameter("approach_window_h",_approach_window_h,0.8);
    psw.getParameter("approach_unknown_th",_approach_unknown_th,10);
	_approach_delta_degree *= M_PI/180.0;


	// - - - - - - - - - - - - - - - - S U B S C R I B E R S - - - - - - - - - - - - - - -
	globalObstacles_sub = priv.subscribe(_obstacles_topic, 1,
			&GoalCalculator::callback_globalObstacles,this);

	_tf_listener = boost::shared_ptr<tf::TransformListener>(
			new tf::TransformListener(ros::Duration(_tf_buffer_size))
	);


	// Publishers
	polygon_pub = priv.advertise<geometry_msgs::PolygonStamped>("polygon_approach", 1,this);
	points_pub = priv.advertise<sensor_msgs::PointCloud>("points_approach", 1,this);

    // - - - - - - - - - - - - - - - - W a i t   f o r   M a p - - - - - - - - - - - - - -

    _get_map_client = priv.serviceClient<nav_msgs::GetMap>(_map_service);
    _make_plan_client = priv.serviceClient<nav_msgs::GetPlan>("make_plan");

    ROS_WARN("Waiting map server");
	while ( ros::ok() && !_get_map_client.waitForExistence(ros::Duration(3.0)) ) ;

    // to prevent execution if ctrl+c is used while waiting
    if (!ros::ok() ) {
    	exit(1);
    }

    // request the map only once!
    if (_is_map_static) {
    	nav_msgs::GetMap map_srv;
    	_get_map_client.call(map_srv);
		_map = map_srv.response.map;
    }
	ROS_INFO("GoalCalculator initialized");
}

GoalCalculator::~GoalCalculator() {

}

/**
 * Used for occupancy checking
 */
void GoalCalculator::callback_globalObstacles(const nav_msgs::GridCells msg) {

	obstacles = msg;
}

geometry_msgs::PoseStamped GoalCalculator::calculeLookGoal(geometry_msgs::PoseStamped current_pose, geometry_msgs::PoseStamped look_pose) {

	geometry_msgs::PoseStamped goal;

    float dx = look_pose.pose.position.x - current_pose.pose.position.x;
    float dy = look_pose.pose.position.y - current_pose.pose.position.y;
    float theta = atan2f(dy,dx);

    goal.header.frame_id = _map_frame;
    goal.header.stamp = ros::Time::now();
    goal.pose.position = current_pose.pose.position;
    goal.pose.orientation.x = 0*sin(0.5*theta);
    goal.pose.orientation.y = 0*sin(0.5*theta);
    goal.pose.orientation.z = 1*sin(0.5*theta);
    goal.pose.orientation.w = 1*cos(0.5*theta);

    return goal;
}


geometry_msgs::PoseStamped GoalCalculator::calculeApproachGoal(geometry_msgs::PoseStamped current_pose, geometry_msgs::PoseStamped approach_pose) {

    std::vector<geometry_msgs::Point> point_vector;

    // update map if necessary
    // only update if enough time has passed.
    if (!_is_map_static) {
		nav_msgs::GetMap map_srv;
		_get_map_client.call(map_srv);
		_map = map_srv.response.map;
	}

    // Calculate angle with respect to goal
    double dx = current_pose.pose.position.x - approach_pose.pose.position.x;
    double dy = current_pose.pose.position.y - approach_pose.pose.position.y;
    double phi = atan2(dy,dx);

    // Generate approach goal candidates
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    bool turn_up = true;
    double radius = _approach_person_radius;	// current_ratio
    double theta = 0.0;							// current angle
    while(radius < _approach_max_radius){

    	geometry_msgs::Point point;
    	point.x = approach_pose.pose.position.x + radius*cosf(phi + theta);
    	point.y = approach_pose.pose.position.y + radius*sinf(phi + theta);
    	point.z = 0.0;

		if(turn_up){
			theta = -theta + _approach_delta_degree;
			if(theta > M_PI){
				theta = 0.0;
				radius += _approach_delta_radius;
			}
			turn_up = false;
		}else{
			if(theta==0 || theta==M_PI){
				turn_up = true;
				continue;
			}
			theta = -theta;
			turn_up = true;
		}

		point_vector.push_back(point);
	}
    ROS_INFO("Generated %lud points candidates for approach ... ",point_vector.size());
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -


	// Publish points if necessary
	if (points_pub.getNumSubscribers() > 0) {

		sensor_msgs::PointCloud cloud;
		cloud.header.frame_id = _map_frame;
		cloud.header.stamp = ros::Time::now();
		cloud.points = array_32(point_vector.size());
		cloud.channels = channel_32(1);
		cloud.channels[0].name = "rgb";
		cloud.channels[0].values = float_array(point_vector.size());

		for (unsigned int k=0; k < point_vector.size(); k++) {

			cloud.points[k].x = point_vector[k].x;
			cloud.points[k].y = point_vector[k].y;
		}
		this->points_pub.publish(cloud);
	}

	// Evaluate point candidates
	std::vector<geometry_msgs::Point>::iterator it;
	for ( it = point_vector.begin(); it < point_vector.end(); it++) {

		// return if candidate is valid.
		if(hasFreeSpace(obstacles, *it)) {

			// Goal Setup
			geometry_msgs::PoseStamped goal_pose;
			goal_pose.pose.position  = *it;

			return calculeLookGoal(goal_pose,approach_pose);
		}
	}
	return current_pose;
}

bool GoalCalculator::hasFreeSpace(nav_msgs::GridCells obstacles, geometry_msgs::Point point) {

	this->publishRectangle(&point,_approach_window_w,_approach_window_h);

	try {

		// Check Inflated Obstacles
		/*
		 * There is no way to tell what point in the array is near
		 * the required point, so we analyze the whole cells.
		 */
		pointArray_it p_it;
		for(p_it = obstacles.cells.begin(); p_it!=obstacles.cells.end(); p_it++){

			// return invalid if a obstacle is found
			if(fabs(point.x - p_it->x) < _approach_obstacle_th
				&& fabs(point.y - p_it->y) < _approach_obstacle_th){

				return false;
			}
		}

		// Check map occupancy < threshold
		/*
		 * We analyze a map portion around the required point!
		 */
		int i1 = ( int)floor((point.x - (_approach_window_w/2) - _map.info.origin.position.x)/_map.info.resolution);
		int i2 = (int)ceil((point.x + (_approach_window_w/2) - _map.info.origin.position.x)/_map.info.resolution);
		int j1 = (int)floor((point.y - (_approach_window_h/2) - _map.info.origin.position.y)/_map.info.resolution);
		int j2 = (int)ceil((point.y + (_approach_window_h/2) - _map.info.origin.position.y)/_map.info.resolution);

		i1 = (int)fmax(0,i1);
		i2 = (int)fmin(_map.info.width-1,i2);
		j1 = (int)fmax(0,j1);
		j2 = (int)fmin(_map.info.height-1,j2);

		/*
		 * Due to random pixels in the image, we cannot expect to have
		 * a perfectly clean window, so we use a free rate threshold
		 */
		int cnt_unknown = 0;
		for(int i=i1;i<=i2;i++){
			for(int j=j1;j<=j2;j++){

				if(_map.data[_map.info.width*j+i]==-1){
					cnt_unknown++;
				}
			}
		}

		// return invalid if there are too many unknown cells
		if(cnt_unknown > _approach_unknown_th){
			return false;
		}

	} catch (...) {
		// TODO: personalized error
		ROS_ERROR("Error on free space checker");
	}

	return true;
}

bool GoalCalculator::transformPose(geometry_msgs::PoseStamped goal, geometry_msgs::PoseStamped &transformed_pose){


	if (goal.header.frame_id == _map_frame) {

		transformed_pose = goal;
		return true;
	}

	try {
		_tf_listener->waitForTransform(_map_frame, goal.header.frame_id, ros::Time::now(), ros::Duration(1.0));
		_tf_listener->transformPose(_map_frame, goal, transformed_pose);

	} catch (tf::TransformException & ex) {

		ROS_ERROR_STREAM("[" << _name << "]: Cannot transform goal because: " << ex.what());

		return false;
	}

	return true;
}

void GoalCalculator::publishRectangle(geometry_msgs::Point* point, double W, double H) {

	if (polygon_pub.getNumSubscribers() > 0) {

		geometry_msgs::PolygonStamped polygon;
		polygon.header.frame_id = "/map";
		polygon.header.stamp = ros::Time::now();
		polygon.polygon.points = array_32(4);
		polygon.polygon.points[0].x = point->x + W/2;
		polygon.polygon.points[0].y = point->y + H/2;
		polygon.polygon.points[1].x = point->x - W/2;
		polygon.polygon.points[1].y = point->y + H/2;
		polygon.polygon.points[2].x = point->x - W/2;
		polygon.polygon.points[2].y = point->y - H/2;
		polygon.polygon.points[3].x = point->x + W/2;
		polygon.polygon.points[3].y = point->y - H/2;
		this->polygon_pub.publish(polygon);
	}
}

} /* namespace uchile_nav */
