#include <cstdlib>
#include <stdlib.h>
#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <bender_utils/ParameterServerWrapper.h>

// ROS services / messages
#include <geometry_msgs/PoseStamped.h>


namespace uchile_nav {

class PosePublisher {

public:
	PosePublisher();
	virtual ~PosePublisher();
	void spin();

private:

	// parameters
	std::string _target_frame;
	std::string _source_frame;
	ros::Duration _tf_tolerance;

	// Listeners
	tf::TransformListener _tf_listener;

	// publishers
	ros::Publisher _pose_pub;

	bool getTransformedPose(geometry_msgs::PoseStamped& msg) {

		geometry_msgs::PoseStamped pose_stamped;
		pose_stamped.header.frame_id = _source_frame;
		pose_stamped.header.stamp = ros::Time::now();
		pose_stamped.pose.orientation.w = 1;

		// transform pose
		try {
			_tf_listener.waitForTransform(_target_frame, _source_frame, pose_stamped.header.stamp, _tf_tolerance);
			_tf_listener.transformPose(_target_frame, pose_stamped, msg);

		} catch (tf::ConnectivityException &e) {
			ROS_WARN_STREAM_THROTTLE(2.0,"Requested a transform between unconnected trees!: " << e.what());
			return false;

		} catch (tf::TransformException &e) {
			ROS_WARN_STREAM_THROTTLE(2.0, "Transform Exception: " << e.what());
			return false;
		}
		return true;
	}

};

PosePublisher::PosePublisher() {

	ros::NodeHandle priv("~");

	// - - - - - - - P A R A M E T E R   S E R V E R - - - - - - - - -
	bender_utils::ParameterServerWrapper psw;
	float transform_tolerance = 0.5;
	psw.getParameter("tf_tolerance",transform_tolerance, transform_tolerance);
	psw.getParameter("target_frame",_target_frame, "/map");
	psw.getParameter("source_frame",_source_frame, "/bender/base_link");
	_tf_tolerance = ros::Duration(transform_tolerance);

	// - - - - - - - P U B L I S H E R S - - - - - - - - - - - - - - -
	_pose_pub = priv.advertise<geometry_msgs::PoseStamped>("pose", 1, this);


	// [important!] wait for timer initialization!
	while (ros::ok() && ros::Time::now().isZero());

}

PosePublisher::~PosePublisher() {}

void PosePublisher::spin() {

	// get current position
	geometry_msgs::PoseStamped robot;
	if ( !getTransformedPose(robot) ) {
		return;
	}
	_pose_pub.publish(robot);
}

}


int main(int argc, char **argv){

	ros::init(argc, argv, "robot_map_pose_publisher");

	boost::scoped_ptr<uchile_nav::PosePublisher> node(
			new uchile_nav::PosePublisher()
	);

	ros::Rate r(30);
	while(ros::ok())
	{
		node->spin();

		r.sleep();
		ros::spinOnce();
	}
	printf("\nQuitting... \n\n");

	return 0;
}
