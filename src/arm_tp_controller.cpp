#include "abstract_tp_controller.hpp"

#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/Pose.h>

namespace tiago_controllers
{

class ArmController : public AbstractController<geometry_msgs::Pose>
{
public:
    ArmController() : AbstractController("arm") { }
};

} // namespace tiago_controllers

using namespace tiago_controllers;

PLUGINLIB_EXPORT_CLASS(ArmController, controller_interface::ControllerBase);
