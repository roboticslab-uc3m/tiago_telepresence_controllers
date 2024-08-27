#include "abstract_tp_controller.hpp"

#include <pluginlib/class_list_macros.h>

namespace tiago_controllers
{

class ArmController : public AbstractController
{
public:
  void update(const ros::Time& time, const ros::Duration& period);
};

} // namespace tiago_controllers

void tiago_controllers::ArmController::update(const ros::Time& time, const ros::Duration& period)
{}

PLUGINLIB_EXPORT_CLASS(tiago_controllers::ArmController, controller_interface::ControllerBase);
