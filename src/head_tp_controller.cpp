#include "generic_tp_controller.hpp"

#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <kdl/frames.hpp>

namespace tiago_controllers
{

class HeadController : public BufferedGenericController<geometry_msgs::QuaternionStamped>
{
public:
    HeadController() : BufferedGenericController("head") { }

protected:
    void processData(const geometry_msgs::QuaternionStamped& msg) override
    {
        const auto rot = KDL::Rotation::Quaternion(msg.quaternion.x, msg.quaternion.y, msg.quaternion.z, msg.quaternion.w);
        double alfa, beta, gamma;
        rot.GetEulerZYX(alfa, beta, gamma);
        accept({-beta, -gamma}, msg.header.stamp);
    }
};

} // namespace tiago_controllers

PLUGINLIB_EXPORT_CLASS(tiago_controllers::HeadController, controller_interface::ControllerBase);
