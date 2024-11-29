#include "generic_tp_controller.hpp"

#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <kdl/frames.hpp>

namespace tiago_controllers
{

class HeadController : public JointBufferController<geometry_msgs::QuaternionStamped>
{
public:
    HeadController() : JointBufferController("head"), q(2) { }

protected:
    void processData(const geometry_msgs::QuaternionStamped& msg) override
    {
        const auto rot = KDL::Rotation::Quaternion(msg.quaternion.x, msg.quaternion.y, msg.quaternion.z, msg.quaternion.w);
        rot.GetEulerZYX(q(0), q(1), _gamma); // R_z_alpha * R_y_beta * R_x_gamma
        accept(q, msg.header.stamp);
    }

private:
    KDL::JntArray q;
    double _gamma; // we are not going to use this
};

} // namespace tiago_controllers

PLUGINLIB_EXPORT_CLASS(tiago_controllers::HeadController, controller_interface::ControllerBase);
