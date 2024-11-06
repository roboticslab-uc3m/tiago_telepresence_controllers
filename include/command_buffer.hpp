#ifndef __COMMAND_BUFFER_HPP__
#define __COMMAND_BUFFER_HPP__

#include <vector>
#include <ros/time.h>
#include <kdl/frames.hpp>

class CommandBuffer
{
public:
    void accept(const std::vector<double> & command);
    std::vector<double> interpolate();
    std::vector<double> getStoredCommand(ros::Time * timestamp = nullptr) const;
    ros::Duration getCommandPeriod() const;
    void reset(const std::vector<double> & initialCommand);

private:
    std::vector<double> storedCommand;
    std::vector<double> interpolationResult;
    ros::Duration commandPeriod;
    ros::Time commandTimestamp;
    ros::Time interpolationTimestamp;

    static const ros::Duration COMMAND_TIMEOUT;
};

#endif // __COMMAND_BUFFER_HPP__
