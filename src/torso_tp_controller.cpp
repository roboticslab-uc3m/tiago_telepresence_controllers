#include "generic_tp_controller.hpp"

#include <pluginlib/class_list_macros.h>
#include <std_msgs/Int32.h>

namespace tiago_controllers
{

class TorsoController : public GenericController<std_msgs::Int32>
{
public:
    TorsoController() : GenericController("torso", true) { }

protected:
    std::vector<double> getDesiredJointValues(const ros::Duration& period) override
    {
        std::lock_guard<std::mutex> lock(mutex);
        return {value.data};
    }
};

} // namespace tiago_controllers

PLUGINLIB_EXPORT_CLASS(tiago_controllers::TorsoController, controller_interface::ControllerBase);
