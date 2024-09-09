#include "generic_tp_controller.hpp"

#include <pluginlib/class_list_macros.h>
#include <std_msgs/Int32.h>

namespace tiago_controllers
{

class GripperController : public GenericController<std_msgs::Int32>
{
public:
    GripperController() : GenericController("gripper", true) { }

    std::vector<double> getDesiredJointValues() override
    {
        std::lock_guard<std::mutex> lock(mutex);
        return {value.data, value.data};
    }
};

} // namespace tiago_controllers

PLUGINLIB_EXPORT_CLASS(tiago_controllers::GripperController, controller_interface::ControllerBase);
