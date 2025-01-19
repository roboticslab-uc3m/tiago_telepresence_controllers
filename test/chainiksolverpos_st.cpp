#include "gtest/gtest.h"

#include <ros/ros.h>

TEST(TestSuite, ChainIkSolverPos_ST_Test)
{
    std::printf("hey\n");
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "chainiksolverpos_st_test");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}
