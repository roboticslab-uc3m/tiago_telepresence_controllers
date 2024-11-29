#ifndef __COMMAND_BUFFER_HPP__
#define __COMMAND_BUFFER_HPP__

#include <iterator> // std::advance
#include <list>
#include <utility> // std::pair
#include <vector>

#include <ros/console.h>
#include <ros/time.h>

constexpr auto CAPACITY_MULTIPLIER = 5;

template <typename T>
class CommandBufferBase
{
public:
    CommandBufferBase(const std::string & name, int minSize, int cmdSize)
        : name(name), minSize(minSize)
    { }

    void accept(const T & command, const ros::SteadyTime & timestamp)
    {
        buffer.emplace_back(command, timestamp);

        if (buffer.size() > minSize * (CAPACITY_MULTIPLIER + 1))
        {
            if (left == buffer.begin())
            {
                // undesirable, but we need to keep our iterators valid
                std::advance(left, 1);
                std::advance(right, 1);
                ROS_WARN("[%s] Reached buffer front", name.c_str());
            }

            buffer.pop_front();

            if (!enabled)
            {
                if (left == right)
                {
                    std::advance(right, 1);
                    left->second = right->second;
                }

                updateSlopes();
                offset = ros::SteadyTime::now() - left->second;
                enabled = true;
            }
        }
    }

    T interpolate()
    {
        const auto refTime = ros::SteadyTime::now() - offset;

        if (enabled && !buffer.empty() && left->second <= refTime)
        {
            bool needsUpdate = false;

            while (right->second < refTime && right != buffer.end())
            {
                std::advance(left, 1);
                std::advance(right, 1);
                needsUpdate = true;
            }

            if (right != buffer.end())
            {
                if (needsUpdate)
                {
                    updateSlopes();
                }

                const auto t = (refTime - left->second).toSec();

                return interpolateInternal(t);
            }
            else
            {
                ROS_WARN("[%s] Reached buffer rear", name.c_str());
                right = left;
            }
        }

        enabled = false;

        return right->first;
    }

    ros::WallDuration getCommandPeriod() const
    {
        return !buffer.empty() ? right->second - left->second : ros::WallDuration(0.0);
    }

    void reset(const T & initialCommand)
    {
        offset.fromSec(0.0);
        buffer.assign(minSize * CAPACITY_MULTIPLIER, std::make_pair(initialCommand, ros::SteadyTime(0.0)));

        left = right = buffer.end();
        std::advance(left, -1);
        std::advance(right, -1);

        enabled = false;

        resetInternal();
    }

protected:
    using BufferType = std::list<std::pair<T, ros::SteadyTime>>;

    virtual void updateSlopes() = 0;
    virtual T interpolateInternal(double t) = 0;
    virtual void resetInternal() { }

    typename BufferType::iterator left;
    typename BufferType::iterator right;

private:
    const std::string name;
    const int minSize {0};
    BufferType buffer;
    ros::WallDuration offset;
    bool enabled {false};
};

class JointCommandBuffer : public CommandBufferBase<std::vector<double>>
{
public:
    JointCommandBuffer(const std::string & name, int minSize, int cmdSize)
        : CommandBufferBase(name, minSize, cmdSize),
          slopes(cmdSize, 0.0)
    { }

protected:
    void updateSlopes() override;
    std::vector<double> interpolateInternal(double t) override;
    void resetInternal() override;

private:
    std::vector<double> slopes;
};

#endif // __COMMAND_BUFFER_HPP__
