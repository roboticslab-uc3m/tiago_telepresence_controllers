#include "controller_base.hpp"

#include <algorithm> // std::max, std::min
#include <urdf/model.h>
#include <std_msgs/Float32MultiArray.h>

#include "tiago_telepresence_controllers/JointPositions.h"

using namespace tiago_controllers;

constexpr auto INPUT_TIMEOUT = 0.25; // [s]

bool ControllerBase::init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle &n)
{
    ROS_INFO("Initializing %s controller", name.c_str());

    std::string robot_desc_string;

    if (!n.getParam("/robot_description", robot_desc_string))
    {
        ROS_ERROR("Could not find robot_description");
        return false;
    }

    urdf::Model model;

    if (!model.initString(robot_desc_string))
    {
        ROS_ERROR("Failed to parse URDF file");
        return false;
    }

    std::vector<std::string> joint_names;

    if (!n.getParam("joint_names", joint_names))
    {
        ROS_ERROR("Could not retrieve joint names");
        return false;
    }

    for (const auto & joint_name : joint_names)
    {
        const auto & joint = model.getJoint(joint_name);

        if (!joint)
        {
            ROS_ERROR("Could not retrieve joint %s", joint_name.c_str());
            return false;
        }

        joints.push_back(hw->getHandle(joint_name));
        jointLimits.emplace_back(joint->limits->lower, joint->limits->upper);
    }

    {
        double statePublishThrottleDouble;

        if (n.getParam("state_publish_throttle", statePublishThrottleDouble))
        {
            statePublishThrottle.fromSec(statePublishThrottleDouble);
        }
    }

    registerSubscriber(n, sub);
    registerPublisher(n, pub);

    {
        std::string out;

        for (const auto & joint : joints)
        {
            out += " " + std::to_string(joint.getPosition());
        }

        ROS_INFO("Initial joint configuration for %s:%s", name.c_str(), out.c_str());
    }

    return additionalSetup(hw, n, robot_desc_string);
}

void ControllerBase::registerPublisher(ros::NodeHandle &n, ros::Publisher &pub)
{
    if (!statePublishThrottle.isZero())
    {
        pub = n.advertise<tiago_telepresence_controllers::JointPositions>("state", 1);
    }
}

void ControllerBase::updateStamp()
{
    std::lock_guard<std::mutex> lock(stampMutex);
    stamp = ros::Time::now();
}

ros::Time ControllerBase::getLastStamp() const
{
    std::lock_guard<std::mutex> lock(stampMutex);
    return stamp;
}

void ControllerBase::update(const ros::Time& time, const ros::Duration& period)
{
    static const ros::Duration timeout(INPUT_TIMEOUT);

    tiago_telepresence_controllers::JointPositions msg;

    for (const auto & joint : joints)
    {
        msg.positions.push_back(joint.getPosition());
    }

    if (!statePublishThrottle.isZero() && time - lastStatePublish > statePublishThrottle)
    {
        updateStatus(msg.status);
        pub.publish(msg);
        lastStatePublish = time;
    }

    if (time - getLastStamp() > timeout)
    {
        if (isActive)
        {
            isActive = false;
            onDisabling();
        }

        return;
    }
    else if (!isActive)
    {
        isActive = true;
        onStarting(msg.positions);
    }

    const auto desired = getDesiredJointValues(msg.positions, period.toSec());

    for (int i = 0; i < joints.size(); i++)
    {
        const auto & limits = jointLimits[i];
        const auto cmd = std::max(limits.first, std::min(limits.second, desired[i]));

        joints[i].setCommand(cmd);
    }
}
