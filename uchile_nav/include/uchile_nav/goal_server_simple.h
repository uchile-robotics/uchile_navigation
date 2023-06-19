/**
 * Simplified version of goal server
 * @author: Gonzalo Olguin
 * @email: golguinm@gmail.com
*/

// TODO: MUCHAS COSAS LOL

#ifndef GOAL_SERVER_SIMPLE_H
#define GOAL_SERVER_SIMPLE_H

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
#include <std_srvs/Empty.h>
#include <uchile_srvs/NavGoal.h>
#include <uchile_srvs/PoseStamped.h>

// Move Base Client
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient; 


class GoalServerSimple
{
    private:
        std::string _name = "goal_server_simple"; 
        ros::NodeHandle nh_;
        std::string _aux_name;
        geometry_msgs::PoseStamped _current_goal;
        geometry_msgs::PoseStamped _current_pose;

        // - - - - - Parameters - - - - - - -
        std::string _map_frame = "map";
        std::string _pose_topic;
        std::string _input_initial_pose_topic;
        std::string _output_initial_pose_topic;
        std::string _base_frame = "bender/base_link";
        double _tf_buffer_size; // seconds
        float _goal_sigma_x;
        float _goal_sigma_y;
        float _goal_sigma_angle;

        // publishers
        ros::Publisher _initial_pose_pub;

        // Listeners
        tf::TransformListener _tf_listener;
        ros::Subscriber _estimated_pose_sub;
        ros::Subscriber _initial_pose_sub;

        // Services
        ros::ServiceServer _go_to_pose_srv;
        ros::ServiceServer _look_to_pose_srv;
        // ros::ServiceServer _approach_to_pose_srv;
        ros::ServiceServer _cancel_goal_srv;
        // ros::ServiceServer _has_arrived_srv;
        ros::ServiceServer _get_current_pose_srv;

        // Move base client
        MoveBaseClient _mbc;

        // done callback stuff
        bool _is_goal_done = false;
        
        void _goalDoneCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr &result) {
            ROS_INFO_STREAM("Goal finished by move_base simple action client");
            _is_goal_done = true;
        }

        geometry_msgs::PoseStamped getCurrentPose();
        move_base_msgs::MoveBaseGoal calculeLookGoal(geometry_msgs::PoseStamped current_pose, move_base_msgs::MoveBaseGoal look_pose);

    public:
        GoalServerSimple(ros::NodeHandle& nh);
        bool sendGoal(move_base_msgs::MoveBaseGoal goal);
        bool goToPose(uchile_srvs::NavGoal::Request &req, uchile_srvs::NavGoal::Response &res);
        bool lookToPose(uchile_srvs::NavGoal::Request &req, uchile_srvs::NavGoal::Response &res);
        bool approachToPose(uchile_srvs::NavGoal::Request &req, uchile_srvs::NavGoal::Response &res);
        bool cancelGoal(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
        bool hasArrived(uchile_srvs::NavGoal::Request &req, uchile_srvs::NavGoal::Response &res);
        bool getCurrentPose(uchile_srvs::PoseStamped::Request &req, uchile_srvs::PoseStamped::Response &res);
        bool transformPose(geometry_msgs::PoseStamped goal, move_base_msgs::MoveBaseGoal& transformed_goal);
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // - - - - - - - S u b s c r i b e r   C a l l b a c k s  -  - - - - - - - - - -
        void callback_newGoal(const geometry_msgs::PoseStamped new_goal);
};


#endif //GOAL_SERVER_SIMPLE_H