#include "command_buffer.hpp"

#include <iterator> // std::advance, std::distance

constexpr auto CAPACITY_MULTIPLIER = 2.0;

// -----------------------------------------------------------------------------

void CommandBuffer::accept(const std::vector<double> & command, const ros::Time & timestamp)
{
    buffer.emplace_back(command, timestamp);

    if (buffer.size() > minSize * CAPACITY_MULTIPLIER)
    {
        buffer.pop_front();

        if (!enabled)
        {
            left = right = buffer.begin();
            std::advance(left, static_cast<int>(minSize * CAPACITY_MULTIPLIER / 2));
            std::advance(right, std::distance(buffer.begin(), left) + 1);
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
        slopes[i] = (right->first[i] - left->first[i]) / dt;
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

            for (size_t i = 0; i < slopes.size(); i++)
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

    if (enabled)
    {
        enabled = false;

        if (left->second <= refTime)
        {
            buffer.resize(minSize, std::make_pair(left->first, left->second));
        }
        else
        {
            buffer.resize(minSize, std::make_pair(right->first, right->second));
        }
    }

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
    buffer.resize(minSize, std::make_pair(initialCommand, ros::Time::now()));
    slopes.resize(initialCommand.size(), 0.0);
    left = right = buffer.begin();
    enabled = false;
}

// -----------------------------------------------------------------------------
