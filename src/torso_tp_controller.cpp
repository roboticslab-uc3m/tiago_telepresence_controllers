#include "generic_tp_controller.hpp"

#include <pluginlib/class_list_macros.h>
#include <std_msgs/Int32.h>

namespace tiago_controllers
{

class TorsoController : public GenericController<std_msgs::Int32>
{
public:
    TorsoController() : GenericController("torso", true) { }

    std::vector<double> getDesiredJointValues() override
    { return {value.data}; }
};

} // namespace tiago_controllers

PLUGINLIB_EXPORT_CLASS(tiago_controllers::TorsoController, controller_interface::ControllerBase);
