#include "generic_tp_controller.hpp"

#include <algorithm> // std::max, std::min
#include <urdf/model.h>

using namespace tiago_controllers;

template <typename T>
GenericController<T>::GenericController(const std::string & _name)
    : name(_name)
{ }

template <typename T>
bool GenericController<T>::init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle &n)
{
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

    if (!n.getParam("step", step))
    {
        ROS_ERROR("Could not retrieve step");
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

    sub = n.subscribe<T>("telepresence/" + name, 1, &GenericController::callback, this);

    return true;
}

template <typename T>
void GenericController<T>::callback(const typename T::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(mutex);
    value = *msg;
}

template <typename T>
void GenericController<T>::update(const ros::Time& time, const ros::Duration& period)
{
    mutex.lock();
    auto localValue = value;
    mutex.unlock();

    for (int i = 0; i < joints.size(); i++)
    {
        const auto & joint = joints[i];
        const auto & limits = jointLimits[i];
        const auto & position = joint.getPosition();

        joint.setCommand(std::max(limits.first, std::min(limits.second, position + step * localValue.data)));
    }
}
