#include "command_buffer.hpp"

#include <kdl/path_line.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/velocityprofile_spline.hpp>

// -----------------------------------------------------------------------------

void JointCommandBuffer::update()
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

void FrameCommandBuffer::update()
{
    auto duration = (right->second - left->second).toSec();

    auto * orient = new KDL::RotationalInterpolation_SingleAxis();
    auto * path = new KDL::Path_Line(left->first.pose, right->first.pose, orient, 0.001);
    auto * profile = new KDL::VelocityProfile_Spline();

    trajectory = std::make_unique<KDL::Trajectory_Segment>(path, profile);
    profile->SetProfileDuration(0, 0.0, 0.0, path->PathLength(), 0.0, 0.0, duration); // TODO
}

// -----------------------------------------------------------------------------

KDL::Frame FrameCommandBuffer::interpolateInternal(double t)
{
    return trajectory->Pos(t);
}

// -----------------------------------------------------------------------------

void FrameCommandBuffer::resetInternal()
{
    trajectory.reset();
}

// -----------------------------------------------------------------------------
