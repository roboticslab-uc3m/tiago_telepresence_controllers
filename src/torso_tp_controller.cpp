#include "abstract_tp_controller.hpp"

#include <pluginlib/class_list_macros.h>

namespace tiago_controllers
{

class TorsoController : public AbstractController
{
public:
  void update(const ros::Time& time, const ros::Duration& period);
};

} // namespace tiago_controllers

void tiago_controllers::TorsoController::update(const ros::Time& time, const ros::Duration& period)
{}

PLUGINLIB_EXPORT_CLASS(tiago_controllers::TorsoController, controller_interface::ControllerBase);
