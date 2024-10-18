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
    bool getDesiredJointValues(const ros::Duration& period, const std::vector<double> & current, std::vector<double> & desired) override
    {
        if (!isSteppingEnabled())
        {
            desired = current;
        }
        else
        {
            desired.resize(current.size());
        }

        return true;
    }

    void registerSubscriber(ros::NodeHandle &n, ros::Subscriber &sub) final override
    {
        sub = n.subscribe<T>("command", 1, &GenericController::callback, this);
    }

    T value;

private:
    void callback(const typename T::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mutex);
        value = *msg;
        updateStamp();
    }
};

} // namespace tiago_controllers

#endif // __TIAGO_GENERIC_TP_CONTROLLER_HPP__
