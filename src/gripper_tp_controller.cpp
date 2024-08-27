#include "abstract_tp_controller.hpp"

#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float32.h>

namespace tiago_controllers
{

class GripperController : public AbstractController<std_msgs::Float32>
{
public:
    GripperController() : AbstractController("gripper") { }
};

} // namespace tiago_controllers

using namespace tiago_controllers;

PLUGINLIB_EXPORT_CLASS(GripperController, controller_interface::ControllerBase);
