#include "generic_tp_controller.hpp"

#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/Pose.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>

namespace tiago_controllers
{

class ArmController : public GenericController<geometry_msgs::Pose>
{
public:
    ArmController() : GenericController("arm", false) { }
    bool additionalSetup(hardware_interface::PositionJointInterface* hw, ros::NodeHandle &n) override;

private:
    KDL::Chain chain;
    KDL::ChainIkSolverVel_pinv * ikSolverVel {nullptr};
    KDL::JntArray q;
};

} // namespace tiago_controllers

bool tiago_controllers::ArmController::additionalSetup(hardware_interface::PositionJointInterface* hw, ros::NodeHandle &n)
{
    KDL::Tree tree;

    if (!kdl_parser::treeFromString(robot_desc_string, tree))
    {
        ROS_ERROR("Failed to construct KDL tree");
        return false;
    }

    std::string start_link, end_link;

    if (!n.getParam("start_link", start_link))
    {
        ROS_ERROR("Could not find start_link parameter");
        return false;
    }

    if (!n.getParam("end_link", end_link))
    {
        ROS_ERROR("Could not find end_link parameter");
        return false;
    }

    if (!tree.getChain(start_link, end_link, chain))
    {
        ROS_ERROR("Failed to get chain from kdl tree");
        return false;
    }

    ROS_INFO("Got chain with %d joints and %d segments", chain.getNrOfJoints(), chain.getNrOfSegments());

    q.resize(chain.getNrOfJoints());

    double eps;
    int maxIter;

    if (!n.getParam("ik_solver_vel_eps", eps))
    {
        ROS_ERROR("Could not find ik_solver_vel_eps parameter");
        return false;
    }

    if (!n.getParam("ik_solver_vel_max_iter", maxIter))
    {
        ROS_ERROR("Could not find ik_solver_vel_max_iter parameter");
        return false;
    }

    ikSolverVel = new KDL::ChainIkSolverVel_pinv(chain, eps, maxIter);

    return true;
}

PLUGINLIB_EXPORT_CLASS(tiago_controllers::ArmController, controller_interface::ControllerBase);
