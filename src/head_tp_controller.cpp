#include "generic_tp_controller.hpp"

#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/Quaternion.h>
#include <kdl/frames.hpp>

namespace tiago_controllers
{

class HeadController : public GenericController<geometry_msgs::Quaternion>
{
public:
    HeadController() : GenericController("head", false) { }

protected:
    std::vector<double> getDesiredJointValues(const ros::Duration& period) override
    {
        mutex.lock();
        auto rot = KDL::Rotation::Quaternion(value.x, value.y, value.z, value.w);
        mutex.unlock();

        double alfa, beta, gamma;
        rot.GetEulerZYX(alfa, beta, gamma);
        return {-beta, -gamma};
    }
};

} // namespace tiago_controllers

PLUGINLIB_EXPORT_CLASS(tiago_controllers::HeadController, controller_interface::ControllerBase);
