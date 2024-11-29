#include "command_buffer.hpp"

// -----------------------------------------------------------------------------

void JointCommandBuffer::updateSlopes()
{
    const auto dt = (right->second - left->second).toSec();

    for (auto i = 0; i < slopes.size(); i++)
    {
        slopes[i] = dt != 0.0 ? (right->first(i) - left->first(i)) / dt : 0.0;
    }
}

// -----------------------------------------------------------------------------

KDL::JntArray JointCommandBuffer::interpolateInternal(double t)
{
    auto out = left->first;

    for (auto i = 0; i < slopes.size(); i++)
    {
        // having y = f(t): f(t+T) = f(t) + T * (delta_y / delta_t)
        out(i) += t * slopes[i];
    }

    return out;
}

// -----------------------------------------------------------------------------

void JointCommandBuffer::resetInternal()
{
    slopes.assign(slopes.size(), 0.0);
}

// -----------------------------------------------------------------------------
