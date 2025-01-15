#include "generic_tp_controller.hpp"

#include <atomic>

#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/PoseStamped.h>

#include <kdl_parser/kdl_parser.hpp>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>

#include "tiago_telepresence_controllers/JointPositions.h"

namespace tiago_telepresence_controllers
{

class ArmController : public FrameBufferController<geometry_msgs::PoseStamped>
{
public:
    ArmController() : FrameBufferController("arm") { }

    ~ArmController() override
    {
        delete fkSolverPos;
        delete ikSolverVel;
    }

    void onStarting(const std::vector<double> & angles) override
    {
        q = jointVectorToKdl(angles);
        fkSolverPos->JntToCart(q, H_0_N_initial);
        ROS_INFO("[%s] Initial position: %f %f %f", getName().c_str(), H_0_N_initial.p.x(), H_0_N_initial.p.y(), H_0_N_initial.p.z());
        H_0_N_prev = H_0_N_initial;
        FrameBufferController::onStarting(angles);
        active = true;
    }

    void onDisabling() override
    {
        active = false;
        status = JointPositions::CMD_OK;
    }

    void updateStatus(int & status) override
    {
        status = this->status;
        this->status = JointPositions::CMD_OK;
    }

protected:
    bool additionalSetup(hardware_interface::PositionJointInterface* hw, ros::NodeHandle &n, const std::string &description) override;
    void processData(const geometry_msgs::PoseStamped& msg) override;
    KDL::Frame convertToBufferType(const std::vector<double> & v) override;
    std::vector<double> convertToVector(const KDL::JntArray & q, const KDL::Frame & H_0_N, double period) override;

private:
    void checkReturnCode(int ret);

    KDL::Chain chain;
    KDL::ChainFkSolverPos_recursive * fkSolverPos {nullptr};
    KDL::ChainIkSolverVel_pinv * ikSolverVel {nullptr};

    KDL::Frame H_0_N_initial;
    KDL::Frame H_0_N_prev;

    KDL::JntArray q;

    std::atomic_bool active {false};
    std::atomic_int32_t status {JointPositions::CMD_OK};
};

} // namespace tiago_telepresence_controllers

using namespace tiago_telepresence_controllers;

bool ArmController::additionalSetup(hardware_interface::PositionJointInterface* hw, ros::NodeHandle &n, const std::string &description)
{
    KDL::Tree tree;

    if (!kdl_parser::treeFromString(description, tree))
    {
        ROS_ERROR("[%s] Failed to construct KDL tree", getName().c_str());
        return false;
    }

    std::string start_link, end_link;

    if (!n.getParam("start_link", start_link))
    {
        ROS_ERROR("[%s] Could not find start_link parameter", getName().c_str());
        return false;
    }

    if (!n.getParam("end_link", end_link))
    {
        ROS_ERROR("[%s] Could not find end_link parameter", getName().c_str());
        return false;
    }

    if (!tree.getChain(start_link, end_link, chain))
    {
        ROS_ERROR("[%s] Failed to get chain from kdl tree", getName().c_str());
        return false;
    }

    ROS_INFO("[%s] Got chain with %d joints and %d segments", getName().c_str(), chain.getNrOfJoints(), chain.getNrOfSegments());

    double epsVel;
    int maxIterVel;

    if (!n.getParam("ik_solver_vel_eps", epsVel))
    {
        ROS_ERROR("[%s] Could not find ik_solver_vel_eps parameter", getName().c_str());
        return false;
    }

    if (!n.getParam("ik_solver_vel_max_iter", maxIterVel))
    {
        ROS_ERROR("[%s] Could not find ik_solver_vel_max_iter parameter", getName().c_str());
        return false;
    }

    fkSolverPos = new KDL::ChainFkSolverPos_recursive(chain);
    ikSolverVel = new KDL::ChainIkSolverVel_pinv(chain, epsVel, maxIterVel);

    return FrameBufferController::additionalSetup(hw, n, description);
}

void ArmController::processData(const geometry_msgs::PoseStamped& msg)
{
    if (active)
    {
        // change in pose between initial and desired, referred to the TCP on its initial pose
        KDL::Frame H_N(
            KDL::Rotation::Quaternion(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w),
            KDL::Vector(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        );

        auto H_0_N = H_0_N_initial * H_N;
        accept(H_0_N, msg.header.stamp);
    }
}

KDL::Frame ArmController::convertToBufferType(const std::vector<double> & v)
{
    const auto q = jointVectorToKdl(v);
    KDL::Frame H;
    fkSolverPos->JntToCart(q, H);
    return H;
}

std::vector<double> ArmController::convertToVector(const KDL::JntArray & q_real, const KDL::Frame & H_0_N, double period)
{
    // refer to base frame, but leave the reference point intact
    const auto twist = H_0_N_prev.M * KDL::diff(H_0_N_prev, H_0_N, period);

    KDL::JntArray qdot(q.rows());
    checkReturnCode(ikSolverVel->CartToJnt(q, twist, qdot));

    for (int i = 0; i < q.rows(); i++)
    {
        q(i) += qdot(i) * period;
    }

    H_0_N_prev = H_0_N;

    return kdlToJointVector(q);
}

void ArmController::checkReturnCode(int ret)
{
    int code;

    switch (ret)
    {
    case KDL::ChainIkSolverVel_pinv::E_CONVERGE_PINV_SINGULAR:
        ROS_WARN_THROTTLE(UPDATE_LOG_THROTTLE, "[%s] Convergence issue: pseudo-inverse is singular", getName().c_str());
        code = JointPositions::CMD_SINGULARITY;
        break;
    case KDL::SolverI::E_SVD_FAILED:
        ROS_ERROR_THROTTLE(UPDATE_LOG_THROTTLE, "[%s] Convergence issue: SVD failed", getName().c_str());
        code = JointPositions::CMD_CONVERGENCE;
        break;
    case KDL::SolverI::E_NOERROR:
        return;
    default:
        ROS_WARN_THROTTLE(UPDATE_LOG_THROTTLE, "[%s] Convergence issue: unknown error", getName().c_str());
        code = JointPositions::CMD_UNKNOWN_ERROR;
        return;
    }

    if (status == JointPositions::CMD_OK || status > code)
    {
        status = code;
    }
}

// don't remove the `tiago_telepresence_controllers` namespace here
PLUGINLIB_EXPORT_CLASS(tiago_telepresence_controllers::ArmController, controller_interface::ControllerBase);
