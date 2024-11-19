#include "command_buffer.hpp"

#include <iterator> // std::advance, std::distance

constexpr auto CAPACITY_MULTIPLIER = 5;

// -----------------------------------------------------------------------------

void CommandBuffer::accept(const std::vector<double> & command, const ros::Time & timestamp)
{
    buffer.emplace_back(command, timestamp);

    if (buffer.size() > minSize  * (CAPACITY_MULTIPLIER + 1))
    {
        if (left == buffer.begin())
        {
            // undesirable, but we need to keep our iterators valid
            std::advance(left, 1);
            std::advance(right, 1);
        }

        buffer.pop_front();

        if (!enabled)
        {
            if (left == right)
            {
                std::advance(right, 1);
            }

            updateSlopes();
            offset = ros::Time::now() - left->second;
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
    const auto refTime = ros::Time::now() - offset;

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
            right = left;
        }
    }

    enabled = false;

    return right->first;
}

// -----------------------------------------------------------------------------

ros::Duration CommandBuffer::getCommandPeriod() const
{
    return !buffer.empty() ? right->second - left->second : ros::Duration(0.0);
}

// -----------------------------------------------------------------------------

void CommandBuffer::reset(const std::vector<double> & initialCommand)
{
    offset.fromSec(0.0);
    buffer.resize(minSize * CAPACITY_MULTIPLIER, std::make_pair(initialCommand, ros::Time::now()));
    slopes.resize(initialCommand.size(), 0.0);
    left = right = buffer.end();
    std::advance(left, -1);
    std::advance(right, -1);
    enabled = false;
}

// -----------------------------------------------------------------------------
