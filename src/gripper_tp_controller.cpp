#include "generic_tp_controller.hpp"

#include <pluginlib/class_list_macros.h>
#include <std_msgs/Int32.h>

namespace tiago_controllers
{

class GripperController : public GenericController<std_msgs::Int32>
{
public:
    GripperController() : GenericController("gripper", true) { }

protected:
    bool getDesiredJointValues(const ros::Duration& period, const std::vector<double> & current, std::vector<double> & desired) override
    {
        std::lock_guard<std::mutex> lock(mutex);
        const auto v = static_cast<double>(value.data);
        desired = {v, v};
        return true;
    }
};

} // namespace tiago_controllers

PLUGINLIB_EXPORT_CLASS(tiago_controllers::GripperController, controller_interface::ControllerBase);
