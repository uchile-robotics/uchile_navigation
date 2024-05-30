#include <uchile_nav/goal_server_simple.h>


GoalServerSimple::GoalServerSimple(ros::NodeHandle& nh)
  : nh_(nh)
  , _mbc("move_base", true)
{

	// _estimated_pose_sub = nh_.subscribe(_pose_topic, 1, &GoalHandler::callback_currentPose,this);
  while(!_mbc.waitForServer(ros::Duration(2.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  ROS_INFO("Move base client initialized");

  // Iitialize services
  _go_to_pose_srv    = nh_.advertiseService("go", &GoalServerSimple::goToPose, this);
  _cancel_goal_srv   = nh_.advertiseService("cancel", &GoalServerSimple::cancelGoal, this);
  _look_to_pose_srv  = nh_.advertiseService("look", &GoalServerSimple::lookToPose, this);
  _get_current_pose_srv = nh_.advertiseService("get_current_pose", &GoalServerSimple::getCurrentPose, this);
  _has_arrived_srv = nh_.advertiseService("has_arrived", &GoalServerSimple::hasArrived, this);

  // Initialize states
  _arrivedState = uchile_nav::GOAL_WAITING;
}

/**
* GOTOPOSE
* @params: Receives a naviga5tion request as a PoseStamped
* @return: True 
*/
bool GoalServerSimple::goToPose(uchile_srvs::NavGoal::Request &req, uchile_srvs::NavGoal::Response &res)
{
  this->_arrivedState = uchile_nav::GOAL_WALKING;
  move_base_msgs::MoveBaseGoal transformed_goal;
  if (!this->transformPose(req.goal, transformed_goal)) {
		return false;
	}

  ROS_INFO("Sending goal");
  
  _mbc.sendGoal(transformed_goal, boost::bind(&GoalServerSimple::_goalDoneCb, this, _1, _2),
			MoveBaseClient::SimpleActiveCallback(),
			MoveBaseClient::SimpleFeedbackCallback());

  _current_goal = req.goal;
//   _mbc.waitForResult();
  
  if(_mbc.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Hooray, the base moved succesfully");
	this->_arrivedState = uchile_nav::GOAL_REACHED;
    return true;
  }

//   ROS_INFO("Robot failed to move");
//   this->_arrivedState = uchile_nav::GOAL_ABORTED;
  return false;
}

bool GoalServerSimple::sendGoal(move_base_msgs::MoveBaseGoal goal) {

	// trick to avoid sending a goal which can be cancelled by cancelGoal()
	this->_arrivedState = uchile_nav::GOAL_WALKING;
	ros::Duration(0.15).sleep();

	_is_goal_done = false;
	_mbc.sendGoal(goal,
			boost::bind(&GoalServerSimple::_goalDoneCb, this, _1, _2),
			MoveBaseClient::SimpleActiveCallback(),
			MoveBaseClient::SimpleFeedbackCallback());
	// _current_goal = goal;

	ROS_INFO("Sending goal (x,y)=(%.2f, %.2f), frame: %s",
			 goal.target_pose.pose.position.x,
			 goal.target_pose.pose.position.y,
			 goal.target_pose.header.frame_id.c_str()
	);

	return true;
}

/**
 * @description: 
 * @param:  
 * @return:
*/
bool GoalServerSimple::cancelGoal(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	try {
		// just cancel past goals
		// _mbc.cancelGoalsAtAndBeforeTime(ros::Time::now()- ros::Duration(0.1));
		_mbc.cancelAllGoals();
		this->_arrivedState = uchile_nav::GOAL_CANCELED;
		ROS_INFO("Goals cancelled");

	} catch (std::exception& e) {

        ROS_WARN_STREAM("Attempt to cancel all goals, when there is no one!: " <<  e.what());
        return false;
	}

	return true;
}

/**
 * @description: 
 * @param:  
 * @return:
*/
bool GoalServerSimple::lookToPose(uchile_srvs::NavGoal::Request &req, uchile_srvs::NavGoal::Response &res)
{
    move_base_msgs::MoveBaseGoal transformed_goal, goal;

	if (!this->transformPose(req.goal, transformed_goal) ) {
		return false;
	}	
	goal = this->calculeLookGoal(this->getCurrentPose(), transformed_goal);

	return this->sendGoal(goal);
}


/**
 * @description: 
 * @param:  
 * @return:
*/
bool GoalServerSimple::getCurrentPose(uchile_srvs::PoseStamped::Request &req, uchile_srvs::PoseStamped::Response &res)
{
	res.pose_out = this->getCurrentPose();
	return true;
}


/**
 * @description: 
 * @param:  
 * @return:
*/
bool GoalServerSimple::hasArrived(uchile_srvs::NavGoal::Request  &req, uchile_srvs::NavGoal::Response &res)
{
	res.state = this->getArrivedState();
    return true;
}

/**
 *******************
 * HELPER METHODS  *
 ******************* 
*/

/**
 * @description: 
 * @param:  
 * @return:
*/
move_base_msgs::MoveBaseGoal GoalServerSimple::calculeLookGoal(geometry_msgs::PoseStamped current_pose, move_base_msgs::MoveBaseGoal look_pose) {

  move_base_msgs::MoveBaseGoal goal;

  float dx = look_pose.target_pose.pose.position.x - current_pose.pose.position.x;
  float dy = look_pose.target_pose.pose.position.y - current_pose.pose.position.y;
  float theta = atan2f(dy, dx);

  goal.target_pose.header.frame_id = _map_frame;
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position = current_pose.pose.position;
  goal.target_pose.pose.orientation.x = 0*sin(0.5*theta);
  goal.target_pose.pose.orientation.y = 0*sin(0.5*theta);
  goal.target_pose.pose.orientation.z = 1*sin(0.5*theta);
  goal.target_pose.pose.orientation.w = 1*cos(0.5*theta);

  return goal;
}

/**
 * @description: Gets the current pose between the robot's base_link
 * and map frame.  
 * @return:
*/
geometry_msgs::PoseStamped GoalServerSimple::getCurrentPose() {

	geometry_msgs::PoseStamped ps, ps_out;

	// Initial Pose in map_frame
	ps.header.frame_id = _base_frame;
	ps.pose.orientation.w = 1;

	bool done = false;
	while (!done && ros::ok()) {
		// transform pose
		try {
			ps.header.stamp = ros::Time::now();
			_tf_listener.waitForTransform(_map_frame, _base_frame, ps.header.stamp, ros::Duration(1.0));
			_tf_listener.transformPose(_map_frame, ps, ps_out);
			done = true;
		} 
		catch (tf::TransformException &e) {
			ROS_WARN_STREAM("Transform Exception: " << e.what());
			done = false;
			ros::Duration(0.1).sleep();
		}
	}
	ps_out.header.stamp = ps.header.stamp;
	ps_out.header.frame_id = _map_frame;
	return ps_out;
}


/**
 * @description: 
 * @param:  
 * @return:
*/
uchile_nav::state_t GoalServerSimple::checkAbortedState() {

	actionlib::SimpleClientGoalState state = _mbc.getState();

	//ROS_WARN_STREAM("MBC STATE: " << state.toString());

	if (   state == actionlib::SimpleClientGoalState::ABORTED
	    || state == actionlib::SimpleClientGoalState::REJECTED) {

		return uchile_nav::GOAL_ABORTED;

	} else if ( state == actionlib::SimpleClientGoalState::PREEMPTED ) {

		return uchile_nav::GOAL_CANCELED;

	}

	return uchile_nav::GOAL_OTHER;
}

/**
 * @description: 
 * @param:  
 * @return:
*/
int GoalServerSimple::getArrivedState() {

	uchile_nav::state_t action_state = this->checkAbortedState();

	if (action_state == uchile_nav::GOAL_ABORTED) {
		return action_state;
	}

	return this->_arrivedState;
}

/**
 * @description: 
 * @param:  
 * @return:
*/
bool GoalServerSimple::transformPose(geometry_msgs::PoseStamped goal, move_base_msgs::MoveBaseGoal& transformed_goal){

	if (goal.header.frame_id == _map_frame) {

		//transformed_pose = goal;
    transformed_goal.target_pose.header = goal.header;
    transformed_goal.target_pose.pose = goal.pose;
		return true;
	}

	try {
    geometry_msgs::PoseStamped transformed_pose;
		_tf_listener.waitForTransform(_map_frame, goal.header.frame_id, ros::Time::now(), ros::Duration(1.0));
		_tf_listener.transformPose(_map_frame, goal, transformed_pose);
    transformed_goal.target_pose.header = transformed_pose.header;
    transformed_goal.target_pose.pose = transformed_pose.pose;

	} catch (tf::TransformException & ex) {

		ROS_ERROR_STREAM("[" << _name << "]: Cannot transform goal because: " << ex.what());

		return false;
	}

	return true;
}


// int main(int argc, char** argv){
//   ros::init(argc, argv, "simple_navigation_goals");
//   ros::NodeHandle nh;
//   GoalServerSimple gs(nh);

//   uchile_srvs::NavGoal goal;
//   goal.request.goal.header.frame_id = "map";
//   goal.request.goal.header.stamp = ros::Time::now();
//   goal.request.goal.pose.position.x = 1.0;
//   goal.request.goal.pose.orientation.w = 1.0;

//   gs.goToPose(goal.request, goal.response);
//   return 0;
// }

int main(int argc, char** argv) {

	ros::init(argc, argv, "goal_server_simple");
    ros::NodeHandle nh;
	GoalServerSimple server_simple(nh);

	ros::Rate r(20);
	int counter = 0;
	uchile_nav::state_t last_arrived_state = server_simple._arrivedState;

	while (ros::ok()) {

		// if (counter < 1) {
		// 	server->update();
		// }
		// counter = (counter+1)%4; // update at // 5 hz
		ros::spinOnce();
		r.sleep();

		if (server_simple._arrivedState != last_arrived_state)
		{
			ROS_INFO("Reached state: %i", (int )server_simple._arrivedState);
			last_arrived_state = server_simple._arrivedState;
		}
	}

	std::cout << "\n\nQuitting ... \n" << std::endl;

	return EXIT_SUCCESS;
}