#ifndef __TIAGO_GENERIC_TP_CONTROLLER_HPP__
#define __TIAGO_GENERIC_TP_CONTROLLER_HPP__

#include <mutex>
#include <string>
#include <utility> // std::pair
#include <vector>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

namespace tiago_controllers
{

template <typename T>
class GenericController : public controller_interface::Controller<hardware_interface::PositionJointInterface>
{
public:
    GenericController(const std::string & name);
    bool init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle &n) override;
    void update(const ros::Time& time, const ros::Duration& period) override;

protected:
    std::string name;
    ros::Subscriber sub;
    std::vector<hardware_interface::JointHandle> joints;
    std::vector<std::pair<double, double>> jointLimits;
    double step {0.0};
    T value;
    mutable std::mutex mutex;

private:
    void callback(const typename T::ConstPtr& msg);
};

} // namespace

#endif // __TIAGO_GENERIC_TP_CONTROLLER_HPP__
