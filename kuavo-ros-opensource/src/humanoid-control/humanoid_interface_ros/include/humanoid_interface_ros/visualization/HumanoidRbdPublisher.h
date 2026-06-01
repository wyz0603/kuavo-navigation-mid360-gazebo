#pragma once

#include <ocs2_core/Types.h>
#include <ocs2_ros_interfaces/mrt/DummyObserver.h>
#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <fstream>

namespace ocs2{
namespace humanoid {

class HumanoidRbdPublisher final : public ocs2::DummyObserver {
 public:
  HumanoidRbdPublisher(ocs2::CentroidalModelRbdConversions& rbdConversions,
                       const ocs2::CentroidalModelInfo& info,
                       const std::string& csvFilePath = "",
                       size_t log_interval = 5);
  ~HumanoidRbdPublisher() override;

  void update(const ocs2::SystemObservation& observation, const ocs2::PrimalSolution& policy,
              const ocs2::CommandData& command) override;

 private:
  ocs2::vector_t rbdStateToDrakeQV(const ocs2::vector_t& rbdState);
  void writeToCSV(const ocs2::vector_t& rbdState, int mode);
  ocs2::vector_t reorderState(const ocs2::vector_t& rbdState) const;

  ocs2::CentroidalModelRbdConversions& rbdConversions_;
  const ocs2::CentroidalModelInfo info_;
  std::ofstream csvFile_;
  bool enableCSV_ = true;

  double lastTime_;
  double writeFrequency_ = 5; 
  size_t msg_count_ = 0;
  size_t log_interval_ = 5;
};

} // namespace humanoid 
} // namespace ocs2
