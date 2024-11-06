#include "generic_tp_controller.hpp"

#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/Quaternion.h>
#include <kdl/frames.hpp>

namespace tiago_controllers
{

class HeadController : public BufferedGenericController<geometry_msgs::Quaternion>
{
public:
    HeadController() : BufferedGenericController("head") { }

protected:
    void processData(const geometry_msgs::Quaternion& msg) override
    {
        auto rot = KDL::Rotation::Quaternion(msg.x, msg.y, msg.z, msg.w);
        double alfa, beta, gamma;
        rot.GetEulerZYX(alfa, beta, gamma);
        accept({-beta, -gamma});
    }
};

} // namespace tiago_controllers

PLUGINLIB_EXPORT_CLASS(tiago_controllers::HeadController, controller_interface::ControllerBase);
