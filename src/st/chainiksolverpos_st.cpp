// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "chainiksolverpos_st.hpp"

#include <vector>

using namespace roboticslab;

namespace
{
    ScrewTheoryIkProblem * buildProblem(const PoeExpression & poe)
    {
        return nullptr; // TODO
    }
}

// -----------------------------------------------------------------------------

ChainIkSolverPos_ST::ChainIkSolverPos_ST(const KDL::Chain & chain, const KDL::JntArray & q_min, const KDL::JntArray & q_max)
    : problem(buildProblem(PoeExpression::fromChain(chain))),
      selector(new ConfigurationSelectorLeastOverallAngularDisplacement(q_min, q_max))
{}

// -----------------------------------------------------------------------------

int ChainIkSolverPos_ST::CartToJnt(const KDL::JntArray & q_init, const KDL::Frame & p_in, KDL::JntArray & q_out)
{
    std::vector<KDL::JntArray> solutions;
    bool ret = problem->solve(p_in, solutions);

    if (!selector->configure(solutions))
    {
        return (error = E_OUT_OF_LIMITS);
    }

    if (!selector->findOptimalConfiguration(q_init))
    {
        return (error = E_OUT_OF_LIMITS);
    }

    selector->retrievePose(q_out);

    return (error = ret ? E_NOERROR : E_NOT_REACHABLE);
}

// -----------------------------------------------------------------------------

const char * ChainIkSolverPos_ST::strError(int error) const
{
    switch (error)
    {
    case E_OUT_OF_LIMITS:
        return "Target pose out of joint limits";
    case E_NOT_REACHABLE:
        return "IK solution not reachable";
    default:
        return KDL::SolverI::strError(error);
    }
}

// -----------------------------------------------------------------------------
