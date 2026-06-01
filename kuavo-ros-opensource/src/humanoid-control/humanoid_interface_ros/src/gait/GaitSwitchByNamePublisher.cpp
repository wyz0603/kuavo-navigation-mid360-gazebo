#include "humanoid_interface_ros/gait/GaitSwitchByNamePublisher.h"

#include <algorithm>

#include <ocs2_core/misc/CommandLine.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_msgs/mode_schedule.h>

#include "humanoid_interface_ros/gait/ModeSequenceTemplateRos.h"

namespace ocs2
{
  namespace humanoid
  {

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    GaitSwitchByNamePublisher::GaitSwitchByNamePublisher(ros::NodeHandle nodeHandle, const std::string &gaitFile, const std::string &robotName,
                                                 bool verbose)
    {
      ROS_INFO_STREAM(robotName + "_mpc_gait_switch_by_name node is setting up ...");
      loadData::loadStdVector(gaitFile, "list", gaitList_, verbose);

      modeSequenceTemplatePublisher_ = nodeHandle.advertise<ocs2_msgs::mode_schedule>(robotName + "_mpc_mode_schedule", 1, true);
      switchGaitByNameSubscriber_ = nodeHandle.subscribe(robotName + "_switch_gait_by_name", 1, &GaitSwitchByNamePublisher::switchGaitByNameCallback, this);
      gaitMap_.clear();
      for (const auto &gaitName : gaitList_)
      {
        gaitMap_.insert({gaitName, loadModeSequenceTemplate(gaitFile, gaitName, verbose)});
      }
      ROS_INFO_STREAM(robotName + "_mpc_gait_switch_by_name command node is ready.");
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    void GaitSwitchByNamePublisher::switchGaitByNameCallback(const kuavo_msgs::switchGaitByName::ConstPtr &msg)
    {
      try
      {
        if (gaitMap_.find(msg->gait_name) == gaitMap_.end()) {
          std::cerr << "[GaitSwitchByNamePublisher] Gait \"" << msg->gait_name << "\" not found.\n";
          return;
        }

        std::cerr << "[GaitSwitchByNamePublisher] Switching to gait \"" << msg->gait_name << "\".\n";
        ModeSequenceTemplate modeSequenceTemplate = gaitMap_.at(msg->gait_name);
        modeSequenceTemplatePublisher_.publish(createModeSequenceTemplateMsg(modeSequenceTemplate));
      }
      catch (const std::exception &e)
      {
        std::cerr << "[GaitSwitchByNamePublisher] \"" << msg->gait_name << "\" not found.\n";
      }
    }
  } // namespace humanoid
} // end of namespace ocs2
