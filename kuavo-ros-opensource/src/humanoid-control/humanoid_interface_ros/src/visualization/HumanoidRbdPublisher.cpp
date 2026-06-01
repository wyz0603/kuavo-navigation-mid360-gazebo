#include "humanoid_interface_ros/visualization/HumanoidRbdPublisher.h"
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

namespace ocs2 {
namespace humanoid {

HumanoidRbdPublisher::HumanoidRbdPublisher(ocs2::CentroidalModelRbdConversions& rbdConversions, 
                                         const ocs2::CentroidalModelInfo& info,
                                         const std::string& csvFilePath,
                                         size_t log_interval)
    : rbdConversions_(rbdConversions), 
      enableCSV_(!csvFilePath.empty()),
      info_(info),
      lastTime_(std::numeric_limits<double>::lowest()),
      log_interval_(log_interval) {
  
  if (enableCSV_) {
    csvFile_.open(csvFilePath);
    if (!csvFile_.is_open()) {
      ROS_ERROR_STREAM("Failed to open CSV file: " << csvFilePath);
      enableCSV_ = false;
    } else {
      std::cout << "open csv file: " << csvFilePath << std::endl;
      // // Write CSV header
      // csvFile_ << "time,";
      // // Base position (3)
      // csvFile_ << "base_pos_x,base_pos_y,base_pos_z,";
      // // Base orientation (3)
      // csvFile_ << "base_ori_x,base_ori_y,base_ori_z,";
      // // Joint positions
     
      csvFile_.flush();
    }
  }
}

HumanoidRbdPublisher::~HumanoidRbdPublisher() {
  if (csvFile_.is_open()) {
    csvFile_.flush();
    csvFile_.close();
  }
}

void HumanoidRbdPublisher::update(const ocs2::SystemObservation& observation, 
                                const ocs2::PrimalSolution& policy,
                                const ocs2::CommandData& command) {

  msg_count_++;
  if (msg_count_ % log_interval_ != 0) return;

  // Convert centroidal state to RBD state
  const auto rbdState = rbdConversions_.computeRbdStateFromCentroidalModel(observation.state, observation.input);
  auto qV = rbdStateToDrakeQV(rbdState);
  // Write to CSV if enabled
  if (enableCSV_) {
    writeToCSV(qV, observation.mode);
  }
}

ocs2::vector_t HumanoidRbdPublisher::rbdStateToDrakeQV(const ocs2::vector_t& rbdState) {
  // 将rbdState转换为Drake的qV格式
  ocs2::vector_t qV(info_.actuatedDofNum*2 + 6 + 7);
  auto rpy = rbdState.segment<3>(0);
  auto angle_zyx = Eigen::Matrix<scalar_t, 3, 1>();
  angle_zyx << rpy;
  auto xyz = rbdState.segment<3>(3);
  auto angular_vel = rbdState.segment<3>(6 + info_.actuatedDofNum);
  auto vel_xyz = rbdState.segment<3>(6 + info_.actuatedDofNum + 3);
  auto quat = ocs2::getQuaternionFromEulerAnglesZyx(angle_zyx);
  auto quat_vec = quat.coeffs(); // 这将返回 (x, y, z, w)
  auto ordered_quat_vec = Eigen::Vector4d(quat_vec[3], quat_vec[0], quat_vec[1], quat_vec[2]); // 重新排列为 (w, x, y, z)
  qV << xyz, ordered_quat_vec, rbdState.segment(6, info_.actuatedDofNum), 
    vel_xyz, angular_vel, rbdState.segment(12 + info_.actuatedDofNum, info_.actuatedDofNum);
  return qV;
}

void HumanoidRbdPublisher::writeToCSV(const ocs2::vector_t& qV, int mode) {
  if (!csvFile_.is_open()) return;

  try {
    csvFile_ << std::fixed << std::setprecision(6);  // 设置浮点数精度
    
    // Map mode values
    int mapped_mode;
    switch(mode) {
      case 15:
        mapped_mode = 0;
        break;
      case 12:
        mapped_mode = 1;
        break;
      case 3:
        mapped_mode = 2;
        break;
      default:
        mapped_mode = -1;  // 对于未知的 mode 值使用 -1
        break;
    }
    
    csvFile_ << mapped_mode << ",";
    
    // Write all state components
    for (int i = 0; i < qV.size() - 1; i++) {
      csvFile_ << qV[i] << ",";
    }
    csvFile_ << qV[qV.size() - 1] << "\n";

    // Only flush periodically
    if (ros::Time::now().toSec() - lastTime_ > 1 / writeFrequency_) {
      csvFile_.flush();
      lastTime_ = ros::Time::now().toSec();
    }

  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("Error writing to CSV file: " << e.what());
  }
}

} // namespace humanoid 
} // namespace ocs2
