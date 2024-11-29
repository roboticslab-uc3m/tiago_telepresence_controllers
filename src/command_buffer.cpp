#include "command_buffer.hpp"

#include <iterator> // std::advance
#include <ros/console.h>

constexpr auto CAPACITY_MULTIPLIER = 5;

// -----------------------------------------------------------------------------

void CommandBuffer::accept(const std::vector<double> & command, const ros::SteadyTime & timestamp)
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

// -----------------------------------------------------------------------------

void CommandBuffer::updateSlopes()
{
    const auto dt = (right->second - left->second).toSec();

    for (auto i = 0; i < slopes.size(); i++)
    {
        slopes[i] = dt != 0.0 ? (right->first[i] - left->first[i]) / dt : 0.0;
    }
}

// -----------------------------------------------------------------------------

std::vector<double> CommandBuffer::interpolate()
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

            const auto T = (refTime - left->second).toSec();
            auto out = left->first;

            for (auto i = 0; i < slopes.size(); i++)
            {
                // having y = f(t): f(t+T) = f(t) + T * (delta_y / delta_t)
                out[i] += T * slopes[i];
            }

            return out;
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

// -----------------------------------------------------------------------------

ros::WallDuration CommandBuffer::getCommandPeriod() const
{
    return !buffer.empty() ? right->second - left->second : ros::WallDuration(0.0);
}

// -----------------------------------------------------------------------------

void CommandBuffer::reset(const std::vector<double> & initialCommand)
{
    offset.fromSec(0.0);

    buffer.clear();
    slopes.clear();

    buffer.resize(minSize * CAPACITY_MULTIPLIER, std::make_pair(initialCommand, ros::SteadyTime(0.0)));
    slopes.resize(initialCommand.size(), 0.0);

    left = right = buffer.end();
    std::advance(left, -1);
    std::advance(right, -1);

    enabled = false;
}

// -----------------------------------------------------------------------------
