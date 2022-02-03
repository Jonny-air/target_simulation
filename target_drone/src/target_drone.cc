#include <target_drone.h>

#include <std_srvs/Empty.h>

namespace target_drone {

TargetDrone::TargetDrone(ros::NodeHandle nh):
    nh_(nh) {
  dt_ = 1.0 / kRate_;
  threshold_ = 1;
  velocity_treshold_= 11;
  // initial position
  pos_ = Eigen::Vector3d(3.0, 3.0, 10.0);
  last_pos_ = pos_;
  // reference position, updated if a reference topic is passed in the launch file
  reference_pos_ = Eigen::Vector3d(0.0, 0.0, 0.0);
  // initial velocity
  velocity_ = 0.0;

  linear_.linear_motion_vector = Eigen::Vector3d(10.0, 0.0, 0.0);

  circular_.theta = 0.0;
  circular_.radius = 0.0;
  circular_.midpoint = Eigen::Vector3d(0.0, 0.0, 0.0);
  circular_.phi_circ = -M_PI;

  eight_.distance = 0;
  eight_.first_midpoint = Eigen::Vector3d(0.0, 0.0, 0.0);
  eight_.second_midpoint = Eigen::Vector3d(0.0, 0.0, 0.0);
  eight_.circle = 2;
  eight_.diag = true;
  eight_.phi = -2.0 * M_PI;
  eight_.phi_0 = 0.0;
  eight_.s = 0.0;
  eight_.x = 0.0;
  eight_.diag_vector = Eigen::Vector3d(0.0, 0.0, 0.0);

  acceleration_ = 5;

  stopped_ = true;
  path_type_ = "";

  // Declaration for all 7 services and publisher
  pub_pos_ = nh_.advertise<geometry_msgs::PointStamped>(
      "pos", 1);

  pub_type_ = nh_.advertise<std_msgs::String>(
      "type", 1);

  pub_vel_ = nh_.advertise<geometry_msgs::Vector3Stamped>(
      "velocity", 1);

  odom_sub = nh_.subscribe("uav_odometry", 1,
      &TargetDrone::OdometryCallback, this);

  service_start_pos_ = nh_.advertiseService(
      "start_pos", &TargetDrone::StartPosCallback, this);

  service_stop_ = nh_.advertiseService(
      "stop", &TargetDrone::StopCallback, this);     

  service_linear_ = nh_.advertiseService(
      "linear", &TargetDrone::LinearCallback, this);

  service_circular_ = nh_.advertiseService(
      "circular", &TargetDrone::CircularCallback, this);

  service_eight_ = nh_.advertiseService(
      "eight", &TargetDrone::EightCallback, this);

  service_caught_ = nh_.advertiseService(
      "caught", &TargetDrone::CaughtCallback, this);
}

// update the reference position
void TargetDrone::OdometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;
  double z = msg->pose.pose.position.z;
  reference_pos_ = Eigen::Vector3d(x, y, z);
}

// set the initial position of the taget drone w.r.t which maneuvers are flown
bool TargetDrone::StartPosCallback(
    target_srvs::TargetDroneStartPos::Request &req,
    target_srvs::TargetDroneStartPos::Response &res) {
  pos_[0] = req.x + reference_pos_[0];
  pos_[1] = req.y + reference_pos_[1];
  pos_[2] = req.z + reference_pos_[2];
  last_pos_= pos_;
  res.success = true;

  if (path_type_ == "caught") {
    path_type_ = "";
  }

  return true;
}

// request to stop motion
bool TargetDrone::StopCallback(
    target_srvs::TargetDroneStop::Request &req,
    target_srvs::TargetDroneStop::Response &res) {
  // slows down until halt
  Accelerate(false, velocity_);
  res.success = true;
  return true;
}

// request linear flight 
bool TargetDrone::LinearCallback(
    target_srvs::TargetDroneLinear::Request &req, 
    target_srvs::TargetDroneLinear::Response &res) {
  if (!stopped_) {
    Accelerate(false, velocity_);
  }
  stopped_ = false;
  path_type_ = "linear";

  acceleration_ = req.acceleration;
  Eigen::Vector3d direction(req.x_direction, req.y_direction, req.z_direction);
  double abs = direction.norm();
  if (abs == 0) {
    stopped_ = true;
  } else {
    linear_.linear_motion_vector = direction / abs;
  }
  Accelerate(true, req.velocity);
  return true;
}

// request circular flight
bool TargetDrone::CircularCallback(
    target_srvs::TargetDroneCircular::Request &req,
    target_srvs::TargetDroneCircular::Response &res) {
  if (!stopped_) {
    Accelerate(false, velocity_);
  }
  stopped_ = false;
  path_type_ = "circular";

  acceleration_ = req.acceleration;
  circular_.theta = double(req.tilt) / 180.0 * M_PI;
  circular_.radius = (double(req.radius) == 0) ? 1 : double(req.radius);
  circular_.phi_circ = -M_PI;

  // midpoint
  double x0 = pos_[0] - circular_.radius * cos(circular_.theta);
  double y0 = pos_[1];
  double z0 = pos_[2] + circular_.radius * sin(circular_.theta);

  circular_.midpoint = Eigen::Vector3d(x0, y0, z0);
  Accelerate(true, req.velocity);
  return true;
}

// request eight flight path
bool TargetDrone::EightCallback(
    target_srvs::TargetDroneEight::Request &req,
    target_srvs::TargetDroneEight::Response &res) {
  if (!stopped_) {
    Accelerate(false, velocity_);
  }
  stopped_ = false;
  path_type_ = "eight";

  acceleration_ = req.acceleration;
  eight_.distance = req.distance;
  eight_.theta = double(req.tilt) / 180.0 * M_PI;
  eight_.radius = (double(req.radius) == 0) ? 1 : double(req.radius);
  eight_.phi_0 = (eight_.distance == 0) ? 0 : 
      acos(2 * eight_.radius / (eight_.distance));
  eight_.s = -req.velocity * dt_;
  eight_.phi = -2.0 * M_PI;
  eight_.circle = 2;
  eight_.diag = true;
  eight_.x = 0.0;

  // midpoint1
  double x0 = pos_[0] + eight_.distance / 2.0 * cos(eight_.theta);
  double y0 = pos_[1];
  double z0 = pos_[2] - eight_.distance / 2.0 * sin(eight_.theta);
  eight_.first_midpoint = Eigen::Vector3d(x0, y0, z0);

  // midpoint2
  double x1 = pos_[0] - eight_.distance / 2.0 * cos(eight_.theta);
  double y1 = pos_[1];
  double z1 = pos_[2] + eight_.distance / 2.0 * sin(eight_.theta);
  eight_.second_midpoint = Eigen::Vector3d(x1, y1, z1);

  Accelerate(true, req.velocity);
  return true;
}

// call this once the drone was caught to move it together with the uav at requested offset, 
// only works if publishing uav position to reference topic
bool TargetDrone::CaughtCallback(target_srvs::TargetDroneCaught::Request &req,
    target_srvs::TargetDroneCaught::Response &res) {
  stopped_ = false;
  path_type_ = "caught";
  caught_offset_[0] = req.offset_x;
  caught_offset_[1] = req.offset_y;
  caught_offset_[2] = req.offset_z;
  return true;
}

void TargetDrone::LinearPath() {
  double ds = velocity_ * dt_;
  pos_[0] += linear_.linear_motion_vector[0] * ds;
  pos_[1] += linear_.linear_motion_vector[1] * ds;
  pos_[2] += linear_.linear_motion_vector[2] * ds;
}

void TargetDrone::CircularPath() {
  double ds = velocity_ * dt_;
  double dphi = ds / circular_.radius;
  pos_[0] = circular_.midpoint[0] - 
      circular_.radius * cos(circular_.theta) * cos(circular_.phi_circ);
  pos_[1] = circular_.midpoint[1] + 
      circular_.radius * cos(circular_.theta) * sin(circular_.phi_circ);
  pos_[2] = circular_.midpoint[2] + 
      circular_.radius * sin(circular_.theta) * cos(circular_.phi_circ);

  circular_.phi_circ += dphi;
}

void TargetDrone::EightPath() {
  double a, b, c;
  double ds = velocity_ * dt_;
  double dphi = ds / eight_.radius;

  // decide if the drone is on the diagonal or on a circle
  if (eight_.circle == 1 && 
      isgreaterequal(eight_.phi, 2 * M_PI - eight_.phi_0) && 
      isless(eight_.phi, 2 * M_PI - eight_.phi_0 + dphi)) {
    // first circle will be finished in the next step
    eight_.phi = 2 * M_PI - eight_.phi_0;
  } else if (eight_.circle == 1 && 
      isgreater(eight_.phi, 2 * M_PI - eight_.phi_0)) { 
    // first circle is finished
    eight_.circle = 2;
    eight_.diag = true;
    eight_.phi = M_PI - eight_.phi_0;

    a = eight_.second_midpoint[0] - 
        eight_.radius * cos(eight_.theta) * cos(eight_.phi) - pos_[0];
    b = eight_.second_midpoint[1] + 
        eight_.radius * cos(eight_.theta) * sin(eight_.phi) - pos_[1];
    c = eight_.second_midpoint[2] + 
        eight_.radius * sin(eight_.theta) * cos(eight_.phi) - pos_[2];
    
    eight_.x = sqrt(pow(a, 2) + pow(b, 2) + pow(c, 2));
    if(eight_.x == 0.0) {
      eight_.diag_vector = Eigen::Vector3d(0.0, 0.0, 0.0);
    } else {
    eight_.diag_vector = 
        Eigen::Vector3d(a / eight_.x, b / eight_.x, c / eight_.x);
    }
  } else if (eight_.circle == 2 && 
      islessequal(eight_.phi, - M_PI + eight_.phi_0) && 
      isgreater(eight_.phi, - M_PI + eight_.phi_0 - dphi)) { 
    // second circle will be finished in the next step
    eight_.phi =  -M_PI + eight_.phi_0;
  } else if (eight_.circle == 2 && 
      islessequal(eight_.phi, -M_PI + eight_.phi_0)) { 
    // second circle is finished
    eight_.circle = 1;
    eight_.diag = true;
    eight_.phi = eight_.phi_0;

    a = eight_.first_midpoint[0] - 
        eight_.radius * cos(eight_.theta) * cos(eight_.phi) - pos_[0];
    b = eight_.first_midpoint[1] + 
        eight_.radius * cos(eight_.theta) * sin(eight_.phi) - pos_[1];
    c = eight_.first_midpoint[2] + 
        eight_.radius * sin(eight_.theta) * cos(eight_.phi) - pos_[2];
    eight_.x = sqrt(pow(a, 2) + pow(b, 2) + pow(c, 2));

    if(eight_.x == 0.0) {
      eight_.diag_vector = Eigen::Vector3d(0.0, 0.0, 0.0);
    } else {
    eight_.diag_vector = 
        Eigen::Vector3d(a / eight_.x, b / eight_.x, c / eight_.x);
    }
  } else if (isgreater(eight_.s, eight_.x - ds)) {
    // diagonal is finished
    eight_.diag = false;
    eight_.s = 0.0;
  }

  // calculating next position
  if (eight_.diag) {
    pos_[0] += eight_.diag_vector[0] * ds;
    pos_[1] += eight_.diag_vector[1] * ds;
    pos_[2] += eight_.diag_vector[2] * ds;
    eight_.s += ds;
  } else if (eight_.circle == 1) {
    pos_[0] = eight_.first_midpoint[0] - 
        eight_.radius * cos(eight_.theta) * cos(eight_.phi);
    pos_[1] = eight_.first_midpoint[1] + 
        eight_.radius * cos(eight_.theta) * sin(eight_.phi);
    pos_[2] = eight_.first_midpoint[2] + 
        eight_.radius * sin(eight_.theta) * cos(eight_.phi);
    eight_.phi += dphi;
  } else if (eight_.circle == 2) {
    pos_[0] = eight_.second_midpoint[0] - 
        eight_.radius * cos(eight_.theta) * cos(eight_.phi);
    pos_[1] = eight_.second_midpoint[1] + 
        eight_.radius * cos(eight_.theta) * sin(eight_.phi);
    pos_[2] = eight_.second_midpoint[2] + 
        eight_.radius * sin(eight_.theta) * cos(eight_.phi);
    eight_.phi -= dphi;
  }
}

void TargetDrone::CaughtPath() {
  pos_[0] = reference_pos_[0] + caught_offset_[0];
  pos_[1] = reference_pos_[1] + caught_offset_[1];
  pos_[2] = reference_pos_[2] + caught_offset_[2];
}

void TargetDrone::PrepareMsg() {
  pos_stamped_.header.stamp = ros::Time::now();
  pos_stamped_.header.frame_id = "world";

  vel_stamped_.header.stamp = ros::Time::now();
  vel_stamped_.header.frame_id = "world";

  if (pos_[2] < threshold_ && !stopped_) {
    ROS_WARN(
        "Target Drone: These parameters result in a z value below the threshold.");
    stopped_ = true;
  } else {
    tf::pointEigenToMsg(pos_, pos_stamped_.point);
  }

  Eigen::Vector3d velocity = (pos_ - last_pos_) / dt_;
  tf::vectorEigenToMsg(velocity, vel_stamped_.vector);
  type_msg_.data = (velocity_ > 0.0) ? path_type_ : "static";

  last_pos_ = pos_;
}

void TargetDrone::Accelerate(bool positive_acceleration, 
    double reference_velocity) {
  int timesteps = abs(reference_velocity) / 
      (acceleration_ * dt_);
  int counter = 0;
  ros::Rate rate(kRate_);
  while (counter <= timesteps) {
    if (positive_acceleration) {
      if (reference_velocity > 0.0) {
        velocity_ += acceleration_ * dt_;
      } else {
        velocity_ -= acceleration_ * dt_;
      }
    } else {
      if (reference_velocity > 0.0) {
        velocity_ -= acceleration_ * dt_;
      } else {
        velocity_ += acceleration_ * dt_;
      }
    }
    GeneratePoint();
    ++counter;
    rate.sleep();
  }
  if (!positive_acceleration) {
    stopped_ = true;
    velocity_ = 0;
  }
  return;
}

void TargetDrone::GeneratePoint()
{
  // a map for the pathType (for the swith cmd)
  std::map<std::string, int> map_path_type;
  map_path_type["linear"] = 0;
  map_path_type["circular"] = 1;
  map_path_type["eight"] = 2;
  map_path_type["caught"] = 3;

  if (!stopped_ && path_type_ != "") {
    switch(map_path_type[path_type_]) {
      case 0:
        LinearPath();
        break;
      case 1:
        CircularPath();
        break;
      case 2:
        EightPath();
        break;
      case 3:
        CaughtPath();
        break;
      default:
        break;
    }
  }
  PrepareMsg();
  pub_pos_.publish(pos_stamped_);
  pub_vel_.publish(vel_stamped_);
  pub_type_.publish(type_msg_);
}

} // namespace target_drone