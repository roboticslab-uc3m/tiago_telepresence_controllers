#include "generic_tp_controller.hpp"

#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float32.h>

namespace tiago_controllers
{

class TorsoController : public GenericController<std_msgs::Float32>
{
public:
    TorsoController() : GenericController("torso") { }
};

} // namespace tiago_controllers

using namespace tiago_controllers;

PLUGINLIB_EXPORT_CLASS(TorsoController, controller_interface::ControllerBase);
