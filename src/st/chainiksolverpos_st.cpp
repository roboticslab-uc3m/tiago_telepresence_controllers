// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "chainiksolverpos_st.hpp"

#include <vector>

#include "TiagoSubproblems.hpp"

using namespace roboticslab;

namespace
{
    KDL::Vector findIntersection(const MatrixExponential & exp1, const MatrixExponential & exp2)
    {
        // "Intersection of Two Lines in Three-Space" by Ronald Goldman, University of Waterloo (Waterloo, Ontario, Canada)
        // published in: "Graphic Gems", edited by Andrew S. Glassner, 1 ed., ch. 5, "3D Geometry" (p. 304)
        // referenced in: https://stackoverflow.com/a/565282

        KDL::Vector cross = exp1.getAxis() * exp2.getAxis();
        KDL::Vector diff = exp2.getOrigin() - exp1.getOrigin();

        double den = KDL::pow(cross.Norm(), 2);
        double t = KDL::dot(cross, diff * exp2.getAxis()) / den;
        double s = KDL::dot(cross, diff * exp1.getAxis()) / den;

        KDL::Vector L1 = exp1.getOrigin() + exp1.getAxis() * t;
        KDL::Vector L2 = exp2.getOrigin() + exp2.getAxis() * s;

        assert(KDL::Equal(L1, L2));

        return L1;
    }

    ScrewTheoryIkProblem * buildProblem(const PoeExpression & poe)
    {
        const auto & exp1 = poe.exponentialAtJoint(0);
        const auto & exp2 = poe.exponentialAtJoint(1);
        const auto & exp3 = poe.exponentialAtJoint(2);
        const auto & exp4 = poe.exponentialAtJoint(3);
        const auto & exp5 = poe.exponentialAtJoint(4);
        const auto & exp6 = poe.exponentialAtJoint(5);
        const auto & exp7 = poe.exponentialAtJoint(6);

        const auto o = exp1.getOrigin() + exp1.getAxis(); // point on axis 1, but not on axis 7
        const auto b = findIntersection(exp2, exp3); // intersection of axes 2 and 3
        const auto d = findIntersection(exp6, exp7); // intersection of axes 6 and 7

        ScrewTheoryIkProblem::Steps steps;
        steps.emplace_back(std::vector<int>{0}, new PadenKahanOne(exp1, d));
        steps.emplace_back(std::vector<int>{3}, new PadenKahanThree(exp4, d, b));
        steps.emplace_back(std::vector<int>{1, 2}, new PadenKahanTwoNormal(exp2, exp3, d, b));
        steps.emplace_back(std::vector<int>{4, 5}, new PadenKahanTwoNormal(exp5, exp6, b, d));
        steps.emplace_back(std::vector<int>{6}, new PadenKahanOne(exp7, o));

        return ScrewTheoryIkProblem::create(poe, steps);
    }
}

// -----------------------------------------------------------------------------

ChainIkSolverPos_ST::ChainIkSolverPos_ST(const KDL::Chain & chain)
    : problem(buildProblem(PoeExpression::fromChain(chain))),
      selector(new TiagoConfigurationSelector)
{}

// -----------------------------------------------------------------------------

int ChainIkSolverPos_ST::CartToJnt(const KDL::JntArray & q_init, const KDL::Frame & p_in, KDL::JntArray & q_out)
{
    std::vector<KDL::JntArray> solutions;
    bool ret = problem->solve(p_in, solutions);

    selector->configure(solutions);
    selector->findOptimalConfiguration(q_init);
    selector->retrievePose(q_out);

    return (error = ret ? E_NOERROR : E_NOT_REACHABLE);
}

// -----------------------------------------------------------------------------

const char * ChainIkSolverPos_ST::strError(int error) const
{
    switch (error)
    {
    case E_NOT_REACHABLE:
        return "IK solution not reachable";
    default:
        return KDL::SolverI::strError(error);
    }
}

// -----------------------------------------------------------------------------
