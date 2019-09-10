/**
 * \file 
 * 
 * Recovery behavior based on executing a particular twist
 *
 * \author Bhaskara Marthi
 * \maintainer Matias Pavez
 */

#ifndef TWIST_RECOVERY_TWIST_RECOVERY_H
#define TWIST_RECOVERY_TWIST_RECOVERY_H

#include <tf/transform_listener.h>
#include <nav_core/recovery_behavior.h>
#include <base_local_planner/costmap_model.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Pose2D.h>
#include <angles/angles.h>

#include <uchile_util/ParameterServerWrapper.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Point.h>

namespace uchile_twist_recovery
{

/// Recovery behavior that takes a given twist and tries to execute it for up to
/// d seconds, or until reaching an obstacle.  
class TwistRecovery : public nav_core::RecoveryBehavior
{
public:
  
  // Doesn't do anything: initialize is where the actual work happens
  TwistRecovery();

  ~TwistRecovery();

  // Initialize the parameters of the behavior
  void initialize (std::string name, tf::TransformListener* tf,
                   costmap_2d::Costmap2DROS* global_costmap,
                   costmap_2d::Costmap2DROS* local_costmap);

  // Run the TwistRecovery recovery behavior.
  void runBehavior();


private:

  // common data for recovery behaviors
  costmap_2d::Costmap2DROS *global_costmap_, *local_costmap_;
  costmap_2d::Costmap2D costmap_;
  std::string name_;
  tf::TransformListener* tf_;
  bool initialized_;
  base_local_planner::CostmapModel* world_model_;

  // parameters
  double duration_;
  double linear_speed_limit_;
  double angular_speed_limit_;
  double linear_acceleration_limit_;
  double angular_acceleration_limit_;
  double controller_frequency_;
  double simulation_inc_;

  geometry_msgs::Twist base_frame_twist_;
  ros::Publisher pub_;
  ros::Publisher pub_rviz_;
  
private:

  geometry_msgs::Pose2D getCurrentLocalPose () const;
  geometry_msgs::Twist scaleGivenAccelerationLimits (const geometry_msgs::Twist& twist, const double time_remaining) const;
  double nonincreasingCostInterval (const geometry_msgs::Pose2D& current, const geometry_msgs::Twist& twist) const;
  double normalizedPoseCost (const geometry_msgs::Pose2D& pose) const;
  geometry_msgs::Twist transformTwist (const geometry_msgs::Pose2D& pose) const;
  geometry_msgs::Pose2D forwardSimulate (const geometry_msgs::Pose2D& p, const geometry_msgs::Twist& twist, const double t=1.0) const;
};

} // namespace twist_recovery

#endif // include guard
