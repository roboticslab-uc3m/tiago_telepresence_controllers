#ifndef __COMMAND_BUFFER_HPP__
#define __COMMAND_BUFFER_HPP__

#include <memory> // std::unique_ptr
#include <utility> // std::pair
#include <vector>

#include <ros/console.h>
#include <ros/time.h>

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/trajectory.hpp>

#include "circular_buffer.hpp"

constexpr auto CAPACITY_MULTIPLIER = 5;

template <typename T>
class CommandBufferBase
{
public:
    using ValueType = T;

    CommandBufferBase(const std::string & name, int minSize)
        : name(name),
          minSize(minSize),
          buffer((minSize + 1) * CAPACITY_MULTIPLIER),
          left(buffer.end()),
          right(buffer.end())
    { }

    virtual ~CommandBufferBase() = default;

    void accept(const T & command, const ros::SteadyTime & timestamp)
    {
        const bool wasAtFront = left == buffer.begin();
        buffer.push_back(std::make_pair(command, timestamp));

        if (buffer.full())
        {
            if (wasAtFront)
            {
                // undesirable, but we need to keep our iterators valid
                ++left;
                ++right;
                ROS_WARN("[%s] Reached buffer front", name.c_str());
            }

            if (!enabled)
            {
                if (left == right)
                {
                    ++right;
                    left->second = right->second;
                }

                update();
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
                ++left;
                ++right;
                needsUpdate = true;
            }

            if (right != buffer.end())
            {
                if (needsUpdate)
                {
                    update();
                }

                const auto t = (refTime - left->second).toSec();
                return interpolateInternal(t);
            }
            else
            {
                ROS_WARN("[%s] Reached buffer back", name.c_str());
                right = left;
            }
        }

        enabled = false;
        return right->first;
    }

    ros::WallDuration getCommandPeriod() const
    {
        return !buffer.empty() ? (right->second - left->second) : ros::WallDuration(0.0);
    }

    void reset(const T & initialCommand)
    {
        const auto dummy = std::make_pair(initialCommand, ros::SteadyTime(0.0));

        offset.fromSec(0.0);
        buffer.clear();

        for (int i = 0; i < minSize * CAPACITY_MULTIPLIER; i++)
        {
            buffer.push_back(dummy);
        }

        left = right = buffer.end() - 1;
        enabled = false;

        resetInternal();
    }

protected:
    using BufferType = CircularBuffer<std::pair<T, ros::SteadyTime>>;

    virtual void update() = 0;
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

class JointCommandBuffer : public CommandBufferBase<KDL::JntArray>
{
public:
    JointCommandBuffer(const std::string & name, int minSize, int cmdSize)
        : CommandBufferBase(name, minSize),
          slopes(cmdSize, 0.0)
    { }

protected:
    void update() override;
    KDL::JntArray interpolateInternal(double t) override;
    void resetInternal() override;

private:
    std::vector<double> slopes;
};

class FrameCommandBuffer : public CommandBufferBase<KDL::Frame>
{
public:
    using CommandBufferBase::CommandBufferBase;

protected:
    void update() override;
    KDL::Frame interpolateInternal(double t) override;
    void resetInternal() override;

private:
    std::unique_ptr<KDL::Trajectory> trajectory;
};

#endif // __COMMAND_BUFFER_HPP__
