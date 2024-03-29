/**
 * \file
 *
 * \author Bhaskara Marthi
 * \maintainer Matias Pavez
 */

#include <pluginlib/class_list_macros.hpp>
#include <tf/transform_datatypes.h>
#include <tf2/utils.h>
#include <uchile_twist_recovery/twist_recovery.h>

// register as a RecoveryBehavior plugin
PLUGINLIB_EXPORT_CLASS(uchile_twist_recovery::TwistRecovery,
                       nav_core::RecoveryBehavior)

namespace gm = geometry_msgs;
namespace vm = visualization_msgs;
namespace cmap = costmap_2d;
namespace blp = base_local_planner;
using std::max;
using std::vector;

namespace uchile_twist_recovery {

TwistRecovery::TwistRecovery()
    : global_costmap_(NULL), local_costmap_(NULL), tf_(NULL),
      initialized_(false) {}

TwistRecovery::~TwistRecovery() { delete world_model_; }

void TwistRecovery::initialize(std::string name, tf2_ros::Buffer *tf,
                               cmap::Costmap2DROS *global_cmap,
                               cmap::Costmap2DROS *local_cmap) {

  ROS_ASSERT(!initialized_);
  name_ = name;
  tf_ = tf;
  local_costmap_ = local_cmap;
  global_costmap_ = global_cmap;
  world_model_ = new blp::CostmapModel(*local_costmap_->getCostmap());

  // advertise publisher
  ros::NodeHandle private_nh("~/" + name);
  pub_ = nh_.advertise<gm::Twist>("cmd_vel", 10);
  pub_rviz_ = nh_.advertise<vm::MarkerArray>("twist_recovery", 1);

  // get some parameters from the parameter server
  uchile_util::ParameterServerWrapper private_psw("~/" + name);
  private_psw.getParameter("linear_x", base_frame_twist_.linear.x, 0.20);
  private_psw.getParameter("linear_y", base_frame_twist_.linear.y, 0.20);
  private_psw.getParameter("angular_z", base_frame_twist_.angular.z, 0.20);
  private_psw.getParameter("duration", duration_, 1.0);

  // control parameters
  uchile_util::ParameterServerWrapper blp_psw("~/TrajectoryPlannerROS");
  blp_psw.getParameter("max_vel_x", linear_speed_limit_, 0.3);
  blp_psw.getParameter("max_vel_theta", angular_speed_limit_, 0.4);
  blp_psw.getParameter("acc_lim_x", linear_acceleration_limit_, 0.3);
  blp_psw.getParameter("acc_lim_theta", angular_acceleration_limit_, 1.5);
  blp_psw.getParameter("controller_frequency", controller_frequency_, 15.0);
  simulation_inc_ = 1 / controller_frequency_;

  ROS_INFO_STREAM_NAMED("top",
                        "Initialized twist recovery with twist (vx,vtheta)="
                            << base_frame_twist_.linear.x << ","
                            << base_frame_twist_.angular.z << ") and duration "
                            << duration_);

  initialized_ = true;
}

gm::Twist scaleTwist(const gm::Twist &twist, const double scale) {
  gm::Twist t;
  t.linear.x = twist.linear.x * scale;
  t.linear.y = twist.linear.y * scale;
  t.angular.z = twist.angular.z * scale;
  return t;
}

gm::Pose2D TwistRecovery::forwardSimulate(const gm::Pose2D &p,
                                          const gm::Twist &twist,
                                          const double t) const {
  gm::Pose2D p2;
  if (t < 0.05) {
    p2.x = p.x + twist.linear.x * t;
    p2.y = 0;
    p2.theta = p.theta + twist.angular.z * t;
    return p2;
  }

  // robot moves using a CP (center point) and a radius of curvature R
  float dS = twist.linear.x * t;  // distance traveled (curved)
  float dA = twist.angular.z * t; // angle turned
  float thetaA = p.theta;         // initial direction

  // normalize angles -pi, pi
  dA = angles::normalize_angle(dA);
  thetaA = angles::normalize_angle(thetaA);

  float AB;
  float beta = M_PI_2 - thetaA;
  if (dA < 0.01) {
    AB = dS;

    // see http://www.wolframalpha.com/input/?i=sin+x/(sqrt(2*(1-cos+x)))
    beta += asinf(1.0);

  } else {
    // distance traveled (straight)
    float tmp = sqrtf(2 * (1 - cosf(dA)));
    AB = dS * tmp / dA;

    beta += asinf(sinf(dA) / tmp);
  }
  beta = angles::normalize_angle(beta);

  ROS_WARN("[recovery_behavior: %s], ", name_.c_str());

  // new pose
  p2.x = p.x - AB * cosf(beta);
  p2.y = p.y + AB * sinf(beta);
  p2.theta = angles::normalize_angle(p.theta + dA);
  return p2;
}

// Return the cost of a pose, modified so that -1 does not equal infinity;
// instead 1e9 does.
double TwistRecovery::normalizedPoseCost(const gm::Pose2D &pose) const {
  const double c = world_model_->footprintCost(
      pose.x, pose.y, pose.theta, local_costmap_->getRobotFootprint(), 0.0,
      0.0);
  return c < 0 ? 1e9 : c;
}

/// Return the maximum d <= duration_ such that starting at the current pose,
/// the cost is nonincreasing for d seconds if we follow twist It might also
/// be good to have a threshold such that we're allowed to have lethal cost
/// for at most the first k of those d seconds, but this is not done
double TwistRecovery::nonincreasingCostInterval(const gm::Pose2D &current,
                                                const gm::Twist &twist) const {
  // -- visualization --
  vm::MarkerArray rviz;
  vm::Marker marker;
  marker.ns = name_;
  marker.lifetime = ros::Duration(0.0);
  marker.header.frame_id = local_costmap_->getGlobalFrameID();
  marker.header.stamp = ros::Time::now();
  marker.action = vm::Marker::MODIFY;
  marker.type = vm::Marker::ARROW;
  marker.scale.x = 0.3;
  marker.scale.y = 0.015;
  marker.scale.z = 0.015;
  std_msgs::ColorRGBA red;
  red.a = 1.0;
  red.r = 1.0;
  red.g = 0.0;
  red.b = 0.0;
  marker.color = red;
  gm::Point p;
  int cnt = 0;
  // --------------------

  double cost = normalizedPoseCost(current);
  double t; // Will hold the first time that is invalid
  for (t = simulation_inc_; t <= duration_; t += simulation_inc_) {
    gm::Pose2D sim = forwardSimulate(current, twist, t);

    // -- visualization --
    marker.id = cnt;
    marker.pose.position.x = sim.x;
    marker.pose.position.y = sim.y;
    marker.pose.orientation.z = sinf(sim.theta / 2.0);
    marker.pose.orientation.w = cosf(sim.theta / 2.0);
    rviz.markers.push_back(marker);
    cnt++;
    // -------------------

    const double next_cost = normalizedPoseCost(sim);
    if (next_cost > cost) {
      // ROS_DEBUG_STREAM_NAMED ("cost", "Cost at " << t << " and pose " <<
      // forwardSimulate(current, twist, t)
      //	<< " is " << next_cost << " which is greater than previous cost
      //" << cost);
      break;
    }
    cost = next_cost;
  }

  // -- visualization --
  if (cnt > 0) {
    pub_rviz_.publish(rviz);
  }
  // -------------------

  return t - simulation_inc_;
}

double linearSpeed(const gm::Twist &twist) {
  // return sqrt(twist.linear.x*twist.linear.x +
  // twist.linear.y*twist.linear.y);
  return fabs(twist.linear.x);
}

double angularSpeed(const gm::Twist &twist) { return fabs(twist.angular.z); }

// Scale twist so we can stop in the given time, and so it's within the max
// velocity
gm::Twist
TwistRecovery::scaleGivenAccelerationLimits(const gm::Twist &twist,
                                            const double time_remaining) const {
  const double linear_speed = linearSpeed(twist);
  const double angular_speed = angularSpeed(twist);
  const double linear_acc_scaling =
      linear_speed / (time_remaining * linear_acceleration_limit_);
  const double angular_acc_scaling =
      angular_speed / (time_remaining * angular_acceleration_limit_);
  const double acc_scaling = max(linear_acc_scaling, angular_acc_scaling);
  const double linear_vel_scaling = linear_speed / linear_speed_limit_;
  const double angular_vel_scaling = angular_speed / angular_speed_limit_;
  const double vel_scaling = max(linear_vel_scaling, angular_vel_scaling);
  return scaleTwist(twist, max(1.0, max(acc_scaling, vel_scaling)));
}

// Get pose in local costmap frame
gm::Pose2D TwistRecovery::getCurrentLocalPose() const {
  gm::PoseStamped p;
  local_costmap_->getRobotPose(p);
  gm::Pose2D pose;
  pose.x = p.pose.position.x;
  pose.y = p.pose.position.y;
  pose.theta = tf2::getYaw(p.pose.orientation);
  return pose;
}

void TwistRecovery::runBehavior() {
  ROS_ASSERT(initialized_);

  if (global_costmap_ == NULL || local_costmap_ == NULL) {
    ROS_ERROR("The costmaps passed to the TwistRecovery object cannot be NULL. "
              "Doing nothing.");
    return;
  }
  ROS_WARN("Twist recovery behavior started.");

  // Figure out how long we can safely run the behavior
  const gm::Pose2D &current = getCurrentLocalPose();
  const double d = nonincreasingCostInterval(current, base_frame_twist_);

  if (d < 0.2) {
    ROS_INFO("[recovery_behavior: %s]. couldn't apply twist (vx,vtheta)=(%.2f, "
             "%.2f).",
             name_.c_str(), base_frame_twist_.linear.x,
             base_frame_twist_.angular.z);
    return;
  }
  ROS_INFO("[recovery_behavior: %s]. applying twist (vx,vtheta)=(%.2f, %.2f) "
           "for %.2f seconds",
           name_.c_str(), base_frame_twist_.linear.x,
           base_frame_twist_.angular.z, d);

  // We'll now apply this twist open-loop for d seconds (scaled so we can
  // guarantee stopping at the end)
  ros::Rate r(controller_frequency_);
  for (double t = 0; t < d; t += 1 / controller_frequency_) {
    // pub_.publish(scaleGivenAccelerationLimits(base_frame_twist_, d - t));
    pub_.publish(base_frame_twist_);
    r.sleep();
  }
  pub_.publish(gm::Twist());
}

} // namespace uchile_twist_recovery
