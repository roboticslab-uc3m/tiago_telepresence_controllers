#ifndef __COMMAND_BUFFER_HPP__
#define __COMMAND_BUFFER_HPP__

#include <list>
#include <utility> // std::pair
#include <vector>
#include <ros/time.h>

class CommandBuffer
{
public:
    CommandBuffer(int minSize, int cmdSize) : minSize(minSize), slopes(cmdSize, 0.0) { }
    void accept(const std::vector<double> & command, const ros::Time & timestamp);
    std::vector<double> interpolate();
    ros::Duration getCommandPeriod() const;
    void reset(const std::vector<double> & initialCommand);

private:
    void updateSlopes();

    const int minSize {0};
    std::list<std::pair<std::vector<double>, ros::Time>> buffer;
    decltype(buffer)::iterator left;
    decltype(buffer)::iterator right;
    std::vector<double> slopes;
    ros::Duration offset;
    bool enabled {false};
};

#endif // __COMMAND_BUFFER_HPP__
