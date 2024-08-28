#include "generic_tp_controller.hpp"

#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/Pose.h>

namespace tiago_controllers
{

class ArmController : public GenericController<geometry_msgs::Pose>
{
public:
    ArmController() : GenericController("arm") { }
};

} // namespace tiago_controllers

PLUGINLIB_EXPORT_CLASS(tiago_controllers::ArmController, controller_interface::ControllerBase);
