#include "abstract_tp_controller.hpp"

#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/Quaternion.h>

namespace tiago_controllers
{

class HeadController : public AbstractController<geometry_msgs::Quaternion>
{
public:
    HeadController() : AbstractController("head") { }
};

} // namespace tiago_controllers

using namespace tiago_controllers;

PLUGINLIB_EXPORT_CLASS(HeadController, controller_interface::ControllerBase);
