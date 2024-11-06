#include "generic_tp_controller.hpp"

#include <pluginlib/class_list_macros.h>
#include <std_msgs/Int32.h>

namespace tiago_controllers
{

class GripperController : public StepperGenericController<std_msgs::Int32>
{
public:
    GripperController() : StepperGenericController("gripper") { }

protected:
    void processData(const std_msgs::Int32& msg) override
    {
        const auto value = static_cast<double>(msg.data);
        accept({value, value});
    }
};

} // namespace tiago_controllers

PLUGINLIB_EXPORT_CLASS(tiago_controllers::GripperController, controller_interface::ControllerBase);
