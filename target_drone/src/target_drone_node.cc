#include <target_drone.h>
#include <map>

int main(int argc, char **argv)
{
  std::string node_name = "target_drone";
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh;
  
  target_drone::TargetDrone target_drone = target_drone::TargetDrone(nh);

  ROS_WARN_STREAM("Target Simulation Node ready");
  ros::Rate loop_rate(target_drone.kRate_);

  while (ros::ok()){
    target_drone.GeneratePoint();
    loop_rate.sleep();
    ros::spinOnce();
  }
  
  return 0;
}

