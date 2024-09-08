#ifndef __TIAGO_GENERIC_TP_CONTROLLER_HPP__
#define __TIAGO_GENERIC_TP_CONTROLLER_HPP__

#include <string>

#include "controller_base.hpp"

namespace tiago_controllers
{

template <typename T>
class GenericController : public ControllerBase
{
public:
    using ControllerBase::ControllerBase;

protected:
    std::vector<double> getDesiredJointValues()
    {
        return std::vector<double>(getJointCount(), 0.0);
    }

    T value;

private:
    void registerSubscriber(ros::NodeHandle &n, ros::Subscriber &sub) override
    {
        sub = n.subscribe<T>("command", 1, &GenericController::callback, this);
    }

    void callback(const typename T::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mutex);
        value = *msg;
        updateStamp();
    }
};

} // namespace tiago_controllers

#endif // __TIAGO_GENERIC_TP_CONTROLLER_HPP__
