#include "generic_tp_controller.hpp"

#include <atomic>

#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/PoseStamped.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/tree.hpp>

constexpr auto UPDATE_LOG_THROTTLE = 1.0; // [s]

namespace tiago_controllers
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
    }

protected:
    bool additionalSetup(hardware_interface::PositionJointInterface* hw, ros::NodeHandle &n, const std::string &description) override;
    void processData(const geometry_msgs::PoseStamped& msg) override;
    KDL::Frame convertToBufferType(const std::vector<double> & v) override;
    std::vector<double> convertToVector(const KDL::Frame & v) override;

private:
    bool checkReturnCode(int ret);

    KDL::Chain chain;
    KDL::ChainFkSolverPos_recursive * fkSolverPos {nullptr};
    KDL::ChainIkSolverVel_pinv * ikSolverVel {nullptr};
    KDL::Frame H_0_N_initial;
    KDL::Frame H_0_N_prev;
    KDL::JntArray q;

    std::atomic_bool active {false};
};

} // namespace tiago_controllers

using namespace tiago_controllers;

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

    double eps;
    int maxIter;

    if (!n.getParam("ik_solver_vel_eps", eps))
    {
        ROS_ERROR("[%s] Could not find ik_solver_vel_eps parameter", getName().c_str());
        return false;
    }

    if (!n.getParam("ik_solver_vel_max_iter", maxIter))
    {
        ROS_ERROR("[%s] Could not find ik_solver_vel_max_iter parameter", getName().c_str());
        return false;
    }

    fkSolverPos = new KDL::ChainFkSolverPos_recursive(chain);
    ikSolverVel = new KDL::ChainIkSolverVel_pinv(chain, eps, maxIter);

    return FrameBufferController::additionalSetup(hw, n, description);
}

void ArmController::processData(const geometry_msgs::PoseStamped& msg)
{
    if (!active)
    {
        return;
    }

    const auto period = getCommandPeriod();

    if (period == 0.0)
    {
        accept(H_0_N_prev, msg.header.stamp);
        return;
    }

    // change in pose between initial and desired, referred to the TCP
    KDL::Frame H_N(
        KDL::Rotation::Quaternion(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w),
        KDL::Vector(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
    );

    auto H_0_N_desired = H_0_N_initial * H_N;
    auto twist = KDL::diff(H_0_N_prev, H_0_N_desired, period);

    // refer to base frame, but leave the reference point intact
    twist = H_0_N_initial.M * twist;

    KDL::JntArray qdot(q.rows());

    if (!checkReturnCode(ikSolverVel->CartToJnt(q, twist, qdot)))
    {
        ROS_WARN_THROTTLE(UPDATE_LOG_THROTTLE, "[%s] Could not calculate joint velocities (1)", getName().c_str());
        accept(H_0_N_prev, msg.header.stamp);
        return;
    }

    const auto & limits = getJointLimits();
    auto q_temp = q;

    for (int i = 0; i < q_temp.rows(); i++)
    {
        q_temp(i) += qdot(i) * period;

        if (q_temp(i) < limits[i].first || q_temp(i) > limits[i].second)
        {
            ROS_WARN("[%s] Joint %d out of limits: %f not in [%f, %f]", getName().c_str(), i, q_temp(i), limits[i].first, limits[i].second);
            accept(H_0_N_prev, msg.header.stamp);
            return;
        }
    }

    KDL::JntArray qdot_temp(chain.getNrOfJoints());

    if (!checkReturnCode(ikSolverVel->CartToJnt(q_temp, twist, qdot_temp)))
    {
        ROS_WARN_THROTTLE(UPDATE_LOG_THROTTLE, "[%s] Could not calculate joint velocities (2)", getName().c_str());
        accept(H_0_N_prev, msg.header.stamp);
        return;
    }

    // no singular point, so update the calculated pose and joint values
    H_0_N_prev = H_0_N_desired;
    q = q_temp;

    accept(H_0_N_desired, msg.header.stamp);
}

KDL::Frame ArmController::convertToBufferType(const std::vector<double> & v)
{
    const auto q = jointVectorToKdl(v);
    KDL::Frame H;
    fkSolverPos->JntToCart(q, H);
    return H;
}

std::vector<double> ArmController::convertToVector(const KDL::Frame & v)
{
    // TODO
}

bool ArmController::checkReturnCode(int ret)
{
    switch (ret)
    {
    case KDL::ChainIkSolverVel_pinv::E_CONVERGE_PINV_SINGULAR:
        ROS_WARN_THROTTLE(UPDATE_LOG_THROTTLE, "[%s] Convergence issue: pseudo-inverse is singular", getName().c_str());
        return false;
    case KDL::SolverI::E_SVD_FAILED:
        ROS_ERROR_THROTTLE(UPDATE_LOG_THROTTLE, "[%s] Convergence issue: SVD failed", getName().c_str());
        return false;
    case KDL::SolverI::E_NOERROR:
        return true;
    default:
        ROS_WARN_THROTTLE(UPDATE_LOG_THROTTLE, "[%s] Convergence issue: unknown error", getName().c_str());
        return false;
    }
}

// don't remove the `tiago_controllers` namespace here
PLUGINLIB_EXPORT_CLASS(tiago_controllers::ArmController, controller_interface::ControllerBase);
