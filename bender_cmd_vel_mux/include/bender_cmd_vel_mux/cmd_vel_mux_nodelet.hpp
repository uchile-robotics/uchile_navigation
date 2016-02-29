/**
* @file /include/bender_cmd_vel_mux/cmd_vel_mux_nodelet.hpp
*
* @brief Structure for the bender_cmd_vel_mux.
*
* License: BSD
* https://raw.github.com/yujinrobot/yujin_ocs/hydro/yocs_cmd_vel_mux/LICENSE
**/
/*****************************************************************************
** Ifdefs
*****************************************************************************/


#ifndef BENDER_VEL_MUX_NODELET_HPP_
#define BENDER_VEL_MUX_NODELET_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <dynamic_reconfigure/server.h>

#include "bender_cmd_vel_mux/reloadConfig.h"
#include "bender_cmd_vel_mux/cmd_vel_subscribers.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace bender_cmd_vel_mux {

/*****************************************************************************
** CmdVelMux
*****************************************************************************/

class CmdVelMuxNodelet : public nodelet::Nodelet
{
public:
  virtual void onInit();

  CmdVelMuxNodelet()
  {
    dynamic_reconfigure_server = NULL;
  }

  ~CmdVelMuxNodelet()
  {
    if (dynamic_reconfigure_server != NULL)
      delete dynamic_reconfigure_server;
  }

private:
  CmdVelSubscribers cmd_vel_sub;    /**< Pool of cmd_vel topics subscribers */
  ros::Publisher mux_cmd_vel_pub;   /**< Multiplexed command velocity topic */
  ros::Publisher active_subscriber; /**< Currently allowed cmd_vel subscriber */

  void timerCallback(const ros::TimerEvent& event, unsigned int idx);
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg, unsigned int idx);

  /*********************
  ** Dynamic Reconfigure
  **********************/
  dynamic_reconfigure::Server<bender_cmd_vel_mux::reloadConfig> * dynamic_reconfigure_server;
  dynamic_reconfigure::Server<bender_cmd_vel_mux::reloadConfig>::CallbackType dynamic_reconfigure_cb;
  void reloadConfiguration(bender_cmd_vel_mux::reloadConfig &config, uint32_t unused_level);

  /*********************
   ** Private Classes
   **********************/
  // Functor assigned to each incoming velocity topic to bind it to cmd_vel callback
  class CmdVelFunctor
  {
  private:
    unsigned int idx;
    CmdVelMuxNodelet* node;

  public:
    CmdVelFunctor(unsigned int idx, CmdVelMuxNodelet* node) :
        idx(idx), node(node)
    {
    }

    void operator()(const geometry_msgs::Twist::ConstPtr& msg)
    {
      node->cmdVelCallback(msg, idx);
    }
  };

  // Functor assigned to each velocity messages source to bind it to timer callback
  class TimerFunctor
  {
  private:
    unsigned int idx;
    CmdVelMuxNodelet* node;

  public:
    TimerFunctor(unsigned int idx, CmdVelMuxNodelet* node) :
        idx(idx), node(node)
    {
    }

    void operator()(const ros::TimerEvent& event)
    {
      node->timerCallback(event, idx);
    }
  };
};

} // namespace bender_cmd_vel_mux

#endif /* BENDER_VEL_MUX_NODELET_HPP_ */
