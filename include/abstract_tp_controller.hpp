#ifndef __TIAGO_ABSTRACT_TP_CONTROLLER_HPP__
#define __TIAGO_ABSTRACT_TP_CONTROLLER_HPP__

#include <utility> // std::pair
#include <vector>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

namespace tiago_controllers
{

class AbstractController : public controller_interface::Controller<hardware_interface::PositionJointInterface>
{
public:
  virtual ~AbstractController() = 0;
  bool init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle &n) override;

protected:
  std::vector<hardware_interface::JointHandle> joints;
  std::vector<std::pair<double, double>> jointLimits;
  double step {0.0};
};

} // namespace

#endif // __TIAGO_ABSTRACT_TP_CONTROLLER_HPP__
