#include "generic_tp_controller.hpp"

#include <atomic>

#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/PoseStamped.h>

#include <kdl_parser/kdl_parser.hpp>

#include <kdl/chainfksolverpos_recursive.hpp>

#include "st/chainiksolverpos_st.hpp"
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
        delete ikSolverPos;
    }

    void onStarting(const std::vector<double> & angles) override
    {
        fkSolverPos->JntToCart(jointVectorToKdl(angles), H_0_N_initial);
        ROS_INFO("[%s] Initial position: %f %f %f", getName().c_str(), H_0_N_initial.p.x(), H_0_N_initial.p.y(), H_0_N_initial.p.z());
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
    bool checkReturnCode(int ret);

    KDL::Frame H_0_N_initial;
    KDL::Chain chain;
    KDL::ChainFkSolverPos_recursive * fkSolverPos {nullptr};
    ChainIkSolverPos_ST * ikSolverPos {nullptr};

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

    KDL::JntArray qMin, qMax;

    const auto & limits = getJointLimits();

    qMin.resize(limits.size());
    qMax.resize(limits.size());

    for (auto i = 0; i < limits.size(); i++)
    {
        qMin(i) = limits[i].first;
        qMax(i) = limits[i].second;
    }

    fkSolverPos = new KDL::ChainFkSolverPos_recursive(chain);
    ikSolverPos = new ChainIkSolverPos_ST(chain, qMin, qMax);

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
    KDL::Frame H;
    fkSolverPos->JntToCart(jointVectorToKdl(v), H);
    return H;
}

std::vector<double> ArmController::convertToVector(const KDL::JntArray & q_real, const KDL::Frame & H_0_N, double period)
{
    KDL::JntArray qd; // its size is set by our solver
    checkReturnCode(ikSolverPos->CartToJnt(q_real, H_0_N, qd));
    return kdlToJointVector(qd);
}

bool ArmController::checkReturnCode(int ret)
{
    if (ret == KDL::SolverI::E_NOERROR)
    {
        return true;
    }

    int code;

    switch (ret)
    {
    case ChainIkSolverPos_ST::E_OUT_OF_LIMITS:
        ROS_WARN_THROTTLE(UPDATE_LOG_THROTTLE, "[%s] Solver solution out of joint limits", getName().c_str());
        code = JointPositions::CMD_OUT_OF_LIMITS;
        break;
    case ChainIkSolverPos_ST::E_NOT_REACHABLE:
        ROS_ERROR_THROTTLE(UPDATE_LOG_THROTTLE, "[%s] Solver solution out of reachable space", getName().c_str());
        code = JointPositions::CMD_UNREACHABLE;
        break;
    default:
        ROS_WARN_THROTTLE(UPDATE_LOG_THROTTLE, "[%s] Unknown solver error", getName().c_str());
        code = JointPositions::CMD_UNKNOWN_ERROR;
        break;
    }

    if (status == JointPositions::CMD_OK || status > code)
    {
        status = code;
    }

    return false;
}

// don't remove the `tiago_telepresence_controllers` namespace here
PLUGINLIB_EXPORT_CLASS(tiago_telepresence_controllers::ArmController, controller_interface::ControllerBase);
