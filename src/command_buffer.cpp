#include "command_buffer.hpp"

const ros::Duration CommandBuffer::COMMAND_TIMEOUT = ros::Duration(1.0); // seconds

// -----------------------------------------------------------------------------

void CommandBuffer::accept(const std::vector<double> & command)
{
    const auto now = ros::Time::now();
    commandPeriod = now - commandTimestamp;
    commandTimestamp = now;
    storedCommand = command;
}

// -----------------------------------------------------------------------------

std::vector<double> CommandBuffer::interpolate()
{
    const auto now = ros::Time::now();
    const auto nextExpectedCommandTimestamp = commandTimestamp + commandPeriod;
    const auto interpolationPeriod = now - interpolationTimestamp;

    if (now > nextExpectedCommandTimestamp || commandPeriod > COMMAND_TIMEOUT)
    {
        // cond. 1: the next command was skipped, it didn't arrive on time or none was received yet
        // cond. 2: the last command was received too long ago, perhaps it was a single step command?
        // in either case, we don't interpolate and just process the last received command (again)
        interpolationResult = storedCommand;
    }
    else
    {
        // note that `interpolationTimestamp` is equal to the `now` value of the previous iteration,
        // hence the reaching of `storedCommand` will be delayed (resulting in a softer slope)
        const auto dt = nextExpectedCommandTimestamp - interpolationTimestamp;
        const auto factor = interpolationPeriod.toSec() / dt.toSec();

        interpolationResult.resize(storedCommand.size());

        for (size_t i = 0; i < storedCommand.size(); i++)
        {
            // having y = f(t): f(t+T) = f(t) + T * (delta_y / delta_t)
            interpolationResult[i] += factor * (storedCommand[i] - interpolationResult[i]);
        }
    }

    interpolationTimestamp = now;
    return interpolationResult;
}

// -----------------------------------------------------------------------------

std::vector<double> CommandBuffer::getStoredCommand(ros::Time * timestamp) const
{
    if (timestamp)
    {
        *timestamp = commandTimestamp;
    }

    return storedCommand;
}

// -----------------------------------------------------------------------------

ros::Duration CommandBuffer::getCommandPeriod() const
{
    return commandPeriod;
}

// -----------------------------------------------------------------------------

void CommandBuffer::reset(const std::vector<double> & initialCommand)
{
    storedCommand = interpolationResult = initialCommand;
    commandPeriod = ros::Duration();
    commandTimestamp = interpolationTimestamp = ros::Time::now();
}

// -----------------------------------------------------------------------------
