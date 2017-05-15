/*
 * GoalServerState.h
 *
 *  Created on: 16-11-2013
 *      Author: matias.pavez.b@gmail.com
 */

#ifndef GOALSERVERSTATE_H_
#define GOALSERVERSTATE_H_

// C, C++
#include <iostream>
#include <boost/shared_ptr.hpp>

// ROS
#include <geometry_msgs/PoseStamped.h>

// Server Helpers
#include "uchile_nav/GoalHandler.h"

namespace uchile_nav {

/*
 * Cannot do #include "GoalServer.h" on this file
 * because a cross include reference would be done, so
 * we use a forward declaration of GoalServer.
 *
 * BUT in order to use GoalServer methods we must do the
 * #include "GoalServer.h" on file GoalServerState.cpp.
 */
class GoalServer;

/* STATE INTERFACE */
class GoalServerState {
public:

	// Empty virtual destructor for proper cleanup
	virtual ~GoalServerState() {};

	virtual void updateTrackState() = 0;

	// State change methods
	virtual bool setGoal(geometry_msgs::PoseStamped goal) = 0;
	virtual bool cancelGoal() = 0;
	virtual bool setQuiet() = 0;
	virtual bool setAlmostReached() = 0;
	virtual bool setReached() = 0;

};

/* INIT STATE */
class InitState : public GoalServerState
{
private:
	GoalServer* server;
	std::string name;

public:

	InitState(uchile_nav::GoalServer* goalServer);
	virtual ~InitState();

	virtual void updateTrackState();

	// State change methods
	virtual bool setGoal(geometry_msgs::PoseStamped goal);
	virtual bool cancelGoal();
	virtual bool setQuiet();
	virtual bool setAlmostReached();
	virtual bool setReached();

};

/* QUIET STATE */
class QuietState : public GoalServerState
{
private:
	GoalServer* server;
	boost::shared_ptr<GoalHandler> handler;

	std::string name;

public:

	QuietState(
			uchile_nav::GoalServer* goalServer,
			boost::shared_ptr<GoalHandler> goalHandler
	);
	virtual ~QuietState();

	virtual void updateTrackState();

	// State change methods
	virtual bool setGoal(geometry_msgs::PoseStamped goal);
	virtual bool cancelGoal();
	virtual bool setQuiet();
	virtual bool setAlmostReached();
	virtual bool setReached();

};

/* WALKING STATE */
class WalkingState : public GoalServerState
{
private:
	GoalServer* server;
	boost::shared_ptr<GoalHandler> handler;

	std::string name;

	// - - - - - Parameters - - - - - - -
	double _goal_almost_reach_radius; // [m]

public:

	WalkingState(
			uchile_nav::GoalServer* goalServer,
			boost::shared_ptr<GoalHandler> goalHandler
	);
	virtual ~WalkingState();

	virtual void updateTrackState();

	// State change methods
	virtual bool setGoal(geometry_msgs::PoseStamped goal);
	virtual bool cancelGoal();
	virtual bool setQuiet();
	virtual bool setAlmostReached();
	virtual bool setReached();

};

/* ALMOST REACH STATE */
class AlmostReachState : public GoalServerState
{
private:
	GoalServer* server;
	boost::shared_ptr<GoalHandler> handler;

	std::string name;

	// - - - - - Parameters - - - - - - -
	std::string _map_frame;
	double _goal_reach_degree_th;

public:

	AlmostReachState(
			uchile_nav::GoalServer* goalServer,
			boost::shared_ptr<GoalHandler> goalHandler
	);
	virtual ~AlmostReachState();

	virtual void updateTrackState();

	// State change methods
	virtual bool setGoal(geometry_msgs::PoseStamped goal);
	virtual bool cancelGoal();
	virtual bool setQuiet();
	virtual bool setAlmostReached();
	virtual bool setReached();

};

} /* namespace uchile_nav */

#endif /* GOALSERVERSTATE_H_ */
