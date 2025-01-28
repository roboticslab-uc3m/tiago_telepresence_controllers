#include "gtest/gtest.h"

#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include "st/chainiksolverpos_st.hpp"

inline KDL::JntArray jointVectorToKdl(const std::vector<double> & v)
{
    // https://forum.kde.org/viewtopic.php%3Ff=74&t=94839.html#p331301
    KDL::JntArray ret;
    ret.data = Eigen::VectorXd::Map(v.data(), v.size());
    return ret;
}

bool doCheck(const KDL::Chain & chain, KDL::ChainFkSolverPos & fkSolverPos, const KDL::JntArray & q)
{
    KDL::Frame H;

    if (fkSolverPos.JntToCart(q, H) < 0)
    {
        ROS_ERROR("JntToCart (1) failed");
        return false;
    }

    ChainIkSolverPos_ST ikSolverPos(chain, H, q);

    KDL::JntArray q_st(chain.getNrOfJoints());

    if (ikSolverPos.CartToJnt(q, H, q_st) != KDL::SolverI::E_NOERROR)
    {
        ROS_ERROR("CartToJnt failed");
        return false;
    }

    for (int i = 0; i < q.rows(); i++)
    {
        if (!KDL::Equal(q(i), q_st(i)))
        {
            ROS_ERROR("q(%d) = %f != q_st(%d) = %f", i, q(i), i, q_st(i));
            return false;
        }
    }

    KDL::Frame H_st;

    if (fkSolverPos.JntToCart(q_st, H_st) < 0)
    {
        ROS_ERROR("JntToCart (2) failed");
        return false;
    }

    if (!KDL::Equal(H, H_st))
    {
        ROS_ERROR("H != H_st");
        return false;
    }

    return true;
}

TEST(TestSuite, ChainIkSolverPos_ST_Test)
{
    ros::NodeHandle n;

    std::string description;
    ASSERT_TRUE(n.getParam("robot_description", description));

    KDL::Tree tree;
    ASSERT_TRUE(kdl_parser::treeFromString(description, tree));

    std::string start_link, end_link;

    ASSERT_TRUE(n.getParam("start_link", start_link));
    ASSERT_TRUE(n.getParam("end_link", end_link));

    KDL::Chain chain;
    ASSERT_TRUE(tree.getChain(start_link, end_link, chain));

    KDL::ChainFkSolverPos_recursive fkSolverPos(chain);
    // ASSERT_TRUE(doCheck(chain, fkSolverPos, jointVectorToKdl({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0})));
    ASSERT_TRUE(doCheck(chain, fkSolverPos, jointVectorToKdl({0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1})));
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "chainiksolverpos_st_test");
    return RUN_ALL_TESTS();
}
