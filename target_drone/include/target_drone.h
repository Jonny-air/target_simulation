#ifndef DROGONE_target_drone_H_
#define DROGONE_target_drone_H_

#include <ros/ros.h>
#include <math.h>
#include <boost/foreach.hpp>

#include <string>
#include <Eigen/Dense>

#include <eigen_conversions/eigen_msg.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>

#include <target_srvs/TargetDroneLinear.h>
#include <target_srvs/TargetDroneStartPos.h>
#include <target_srvs/TargetDroneStop.h>
#include <target_srvs/TargetDroneCircular.h>
#include <target_srvs/TargetDroneEight.h>
#include <target_srvs/TargetDroneCaught.h>

#define foreach BOOST_FOREACH

namespace target_drone {

/**
 * @brief The parameters for the linear path
 * 
 */
struct LinearParam
{
  Eigen::Vector3d linear_motion_vector;
};

/**
 * @brief The parameters for the circular path
 * 
 */
struct CircularParam
{
  double theta; // the angle of the circle relative to y
  double radius; 
  double phi_circ; // starting point for the circular movement
  Eigen::Vector3d midpoint;
};

/**
 * @brief The parameters for the eight figure path
 * 
 */
struct EightParam
{
  double theta;
  double radius;
  int distance;
  Eigen::Vector3d first_midpoint;
  Eigen::Vector3d second_midpoint;
  bool diag; // true if the drone is on the diagonal line between the circles
  int circle; // defines on which circle the drone is
  double phi; // the progress of the circle
  double phi_0; // starting angle
  double s; // progress of the diagonal
  double x; // length of the diagonal
  Eigen::Vector3d diag_vector; // vector of diag
};

/**
 * @brief The main class of the victim drone node
 * 
 */
class TargetDrone {
 public:
  /**
  * @brief Construct a new Victim Drone object
  * 
  * @param nh nodehandle of the victim drone node
  */
  TargetDrone(ros::NodeHandle nh);

  /**
   * @brief Main function of the class. Decides which movement is wanted and 
   * calls the corresponding function
   * 
   */
  void GeneratePoint();

  // needs to be public such that it can be accessed in the node
  int kRate_ = 30;
  
 private:
  /**
   * @brief The callback function for DroGone's odometry
   * 
   * @param msg msg containing the position of our uav
   */
  void OdometryCallback(const nav_msgs::Odometry::ConstPtr& msg);

  /**
   * @brief The callback function for a change starting position service call
   * 
   * @param req the request data sent from the gui
   * @param res the response to the gui 
   * @return true Always returns true
   * @return false 
   */
  bool StartPosCallback(target_srvs::TargetDroneStartPos::Request &req,
      target_srvs::TargetDroneStartPos::Response &res);

  /**
   * @brief The callback function for a stop service call
   * 
   * @param req the request data sent from the gui
   * @param res the response to the gui 
   * @return true Always returns true
   * @return false 
   */
  bool StopCallback(target_srvs::TargetDroneStop::Request &req, 
      target_srvs::TargetDroneStop::Response &res);

  /**
   * @brief The callback function for a linear movement service call
   * 
   * @param req the request data sent from the gui
   * @param res the response to the gui 
   * @return true Always returns true
   * @return false 
   */
  bool LinearCallback(target_srvs::TargetDroneLinear::Request &req, 
      target_srvs::TargetDroneLinear::Response &res);

  /**
   * @brief The callback function for a circular movement service call
   * 
   * @param req the request data sent from the gui
   * @param res the response to the gui 
   * @return true Always returns true
   * @return false 
   */
  bool CircularCallback(target_srvs::TargetDroneCircular::Request &req,
      target_srvs::TargetDroneCircular::Response &res);

  /**
   * @brief The callback function for a eight figure movement service call
   * 
   * @param req the request data sent from the gui
   * @param res the response to the gui 
   * @return true Always returns true
   * @return false 
   */
  bool EightCallback(target_srvs::TargetDroneEight::Request &req,
      target_srvs::TargetDroneEight::Response &res);

  /**
   * @brief The function for the case that the drone is caught
   * 
   * @param req the request data containing the offset sent from check for catch
   * @param res the empty response to check for catch
   * @return true Always returns true
   * @return false 
   */
  bool CaughtCallback(target_srvs::TargetDroneCaught::Request &req,
      target_srvs::TargetDroneCaught::Response &res);

  /**
   * @brief Adds a timestamp and frame id to the msg
   * 
   */
  void PrepareMsg();
  
  /**
   * @brief Moves one timestep in linear direction
   * 
   */
  void LinearPath();

  /**
   * @brief Moves one timestep in circular direction
   * 
   */
  void CircularPath();

  /**
   * @brief Moves one timestep on the figure eight
   * 
   */
  void EightPath();

  /**
   * @brief Makes the victim drone go with drogone once its caught
   * 
   */
  void CaughtPath();

  /**
   * @brief Starts and stops the victim drone in a realistic way by 
   * changing the velocity
   * 
   */
  void ChangeVelocity();

  void Accelerate(bool positive_acceleration, double reference_velocity);

  ros::NodeHandle nh_;
  ros::Publisher pub_pos_;
  ros::Publisher pub_type_;
  ros::Publisher pub_vel_;
  ros::Subscriber odom_sub;
  ros::ServiceServer service_start_pos_;
  ros::ServiceServer service_stop_;
  ros::ServiceServer service_linear_;
  ros::ServiceServer service_circular_;
  ros::ServiceServer service_eight_;
  ros::ServiceServer service_caught_;

  float dt_;
  int threshold_; // lowest acceptable z
  double velocity_treshold_;
  Eigen::Vector3d pos_;
  Eigen::Vector3d last_pos_;
  geometry_msgs::PointStamped pos_stamped_;
  geometry_msgs::Vector3Stamped vel_stamped_;
  std_msgs::String type_msg_;
  Eigen::Vector3d reference_pos_;
  bool stopped_; 
  std::string path_type_;
  double velocity_;
  double acceleration_;
  Eigen::Vector3d caught_offset_;

  LinearParam linear_;
  CircularParam circular_;
  EightParam eight_;
};  

} // namespace target_drone

#endif // DROGONE_target_drone_H_