/*
 * GoalStates.h
 *
 *      Author: matias.pavez.b@gmail.com
 */

#ifndef GOALSTATES_H_
#define GOALSTATES_H_

namespace uchile_nav {

enum state_t {
	GOAL_WAITING = 0,
	GOAL_WALKING = 1,
	GOAL_ALMOST_REACHED = 2,
	GOAL_REACHED = 3,
	GOAL_ABORTED = 4,
	GOAL_CANCELED = 5,
	GOAL_OTHER = 10
};



} /* namespace uchile_nav */

#endif /* GOALSERVERSTATE_H_ */
