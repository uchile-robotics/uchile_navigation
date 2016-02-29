#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <bender_utils/ParameterServerWrapper.h>
#include <geometry_msgs/Twist.h>


// expande las partículas de amcl constantemente, para hacer más robusta la localización
// cuando el robot está avanzando muy lento(o quieto), no se expanden las particulas
// TODO: expandir mucho las partículas cuando el robot esté fuera del mapa

geometry_msgs::PoseWithCovarianceStamped current_pose;
tf::TransformListener* tf_listener;
ros::Publisher amcl_pub;
bool should_publish = false;
float linear_th;
float angular_th;

void spreadAMCL(float sigma_x, float sigma_y, float sigma_angle){

	ROS_INFO("spreading amcl pose . . .");
	try{
		geometry_msgs::PoseWithCovarianceStamped ini_pose;
		ini_pose.header.stamp = ros::Time::now();
		ini_pose.header.frame_id = "/map";

		// build matrix
		ini_pose.pose.covariance[6*0+0] = sigma_x * sigma_x;
		ini_pose.pose.covariance[6*1+1] = sigma_y * sigma_y;
		ini_pose.pose.covariance[6*5+5] = sigma_angle * sigma_angle;

		// transform (0,0) in order to get the current pose in map
		tf::StampedTransform transform;
		try{
			tf_listener->lookupTransform("/map", "/bender/base_link",ros::Time(0), transform);
		}catch (tf::TransformException ex){
			ROS_WARN("En initial_pose: %s",ex.what());
		}
		ini_pose.pose.pose.position.x = transform.getOrigin().x();
		ini_pose.pose.pose.position.y = transform.getOrigin().y();
		ini_pose.pose.pose.position.z = transform.getOrigin().z();
		ini_pose.pose.pose.orientation.x = transform.getRotation().x();
		ini_pose.pose.pose.orientation.y = transform.getRotation().y();
		ini_pose.pose.pose.orientation.z = transform.getRotation().z();
		ini_pose.pose.pose.orientation.w = transform.getRotation().w();

		// publish on initial pose topic (spread)
		amcl_pub.publish(ini_pose);

	}catch (std::exception &e) {
		ROS_ERROR_STREAM("Failed to spread amcl. " << e.what());
	}
}

void vel_command_callback(const geometry_msgs::Twist &msg) {

	if (   fabsf(msg.linear.x)  > linear_th
		|| fabsf(msg.angular.z) > angular_th) {

		if (should_publish == false) {
			// disp info if necessary
			should_publish = true;
			ROS_INFO_STREAM("amcl spreading started");
		}

	} else {
		// disp info if necessary
		if (should_publish == true) {
			should_publish = false;
			ROS_INFO_STREAM("amcl spreading stopped");
		}
	}
}

int main(int argc, char **argv){

	ros::init(argc, argv, "amcl_spreader");
	ros::NodeHandle priv("~");

	float dt; // [s]
	float sigma_xy;
	float sigma_angle;

	// parameters
	bender_utils::ParameterServerWrapper psw;
	psw.getParameter("dt", dt, 3);
	psw.getParameter("sigma_xy", sigma_xy, 0.1);
	psw.getParameter("sigma_angle", sigma_angle, M_PI/6.0);
	psw.getParameter("linear_th", linear_th, 0.05);
	psw.getParameter("angular_th", angular_th, 0.1);
	// rviz usa: sigma_xy = 0.5; sigma_angle = M_PI/12.0;
	ros::Duration publish_period(dt);

	// subscriber
	ros::Subscriber vel_sub = priv.subscribe("cmd_vel", 1, &vel_command_callback);

	// publisher
	tf_listener = new tf::TransformListener(ros::Duration(5));
	amcl_pub = priv.advertise<geometry_msgs::PoseWithCovarianceStamped>("/bender/nav/initialpose", 10);


	// 20[Hz]
	ros::Rate r(20);
	ROS_INFO("Ready to work");

	ros::Time last_pub_attempt = ros::Time::now();
	ros::Time now;
	while(ros::ok())
	{
		now = ros::Time::now();
		if (now-last_pub_attempt > publish_period) {

			if (should_publish) {
				spreadAMCL(sigma_xy,sigma_xy,sigma_angle);
			}
			last_pub_attempt = now;
		}

		// process callbacks
		ros::spinOnce();
		r.sleep();
	}
	printf("\nQuitting... \n\n");

	delete tf_listener;

	return 0;
}
