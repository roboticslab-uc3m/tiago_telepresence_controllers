#include "generic_tp_controller.hpp"

#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/Quaternion.h>

namespace tiago_controllers
{

class HeadController : public GenericController<geometry_msgs::Quaternion>
{
public:
    HeadController() : GenericController("head") { }
};

} // namespace tiago_controllers

PLUGINLIB_EXPORT_CLASS(tiago_controllers::HeadController, controller_interface::ControllerBase);
