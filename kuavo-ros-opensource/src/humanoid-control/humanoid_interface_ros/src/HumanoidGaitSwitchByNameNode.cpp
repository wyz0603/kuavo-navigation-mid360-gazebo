#include <ros/init.h>
#include <ros/package.h>

#include "humanoid_interface_ros/gait/GaitSwitchByNamePublisher.h"

using namespace ocs2;
using namespace humanoid;

int main(int argc, char *argv[])
{
  const std::string robotName = "humanoid";

  // Initialize ros node
  ros::init(argc, argv, robotName + "_gait_switch_by_name");
  ros::NodeHandle nodeHandle;
  // Get node parameters
  std::string gaitCommandFile;
  nodeHandle.getParam("/gaitCommandFile", gaitCommandFile);
  std::cerr << "Loading gait file: " << gaitCommandFile << std::endl;

  GaitSwitchByNamePublisher gaitSwitchPublisher(nodeHandle, gaitCommandFile, robotName, false);

  ::ros::spin();

  // Successful exit
  return 0;
}
