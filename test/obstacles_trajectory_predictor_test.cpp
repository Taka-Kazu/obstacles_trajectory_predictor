#include <gtest/gtest.h>

#include <ros/ros.h>

#include "obstacles_trajectory_predictor/obstacle.h"
#include "obstacles_trajectory_predictor/social_force_model.h"
#include "obstacles_trajectory_predictor/obstacles_tracker.h"

TEST(TestSuite, test0)
{
    Eigen::Vector2d p(0, 0);
    Eigen::Vector2d f(10, 0);
    Obstacle o(p);
    Eigen::Vector4d x1 = o.get_next_state(o.x, f, 10);
    std::cout << x1 << std::endl;;
    EXPECT_NEAR(8.333, x1(0), 0.001);
    EXPECT_NEAR(0, x1(1), 0.001);
    EXPECT_NEAR(1.666, x1(2), 0.001);
    EXPECT_NEAR(0, x1(3), 0.001);
}

TEST(TestSuite, test1)
{
    Eigen::Vector2d p(0, 0);
    Eigen::Vector2d f(10, 0);
    Obstacle o(p);
    o.predict(f, 1.0);
    std::cout << o.x << std::endl;
    EXPECT_NEAR(0.08333, o.x(0), 0.001);
    EXPECT_NEAR(0, o.x(1), 0.001);
    EXPECT_NEAR(0.1666, o.x(2), 0.001);
    EXPECT_NEAR(0, o.x(3), 0.001);
}

TEST(TestSuite, test2)
{
    Eigen::Vector2d p(0, 0);
    Eigen::Vector2d f(10, 1);
    double dt = 0.1;
    Obstacle o(p);
    std::cout << "i: " << 0 << "\nx\n" << o.x.transpose() << "\np:\n" << o.p << std::endl;
    for(int i=1;i<10;i++){
        o.predict(f, dt);
        std::cout << "i: " << i << "\nx\n" << o.x.transpose() << "\np:\n" << o.p << std::endl;
    }
    EXPECT_NEAR(0.0675, o.x(0), 0.001);
}

TEST(TestSuite, test3)
{
    SocialForceModel sfm;
    std::vector<Obstacle> agents;
    agents.push_back(Obstacle(Eigen::Vector2d(0, 0)));
    agents.push_back(Obstacle(Eigen::Vector2d(0, 1)));
    agents.push_back(Obstacle(Eigen::Vector2d(5, -1)));
    sfm.set_observed_agents_state(agents);
    std::vector<Eigen::Vector2d> forces;
    for(size_t i=0;i<agents.size();i++){
        Eigen::Vector2d f = sfm.get_social_force(i);
        std::cout << "f" << i << ": " << f.transpose() << std::endl;
        forces.push_back(f);
    }
    EXPECT_GT(0, forces[0](1));
    EXPECT_LT(0, forces[1](1));
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);

    ros::init(argc, argv, "state_lattice_planner_test");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Duration(1.0).sleep();

    int r_e_t = RUN_ALL_TESTS();

    spinner.stop();

    ros::shutdown();

    return r_e_t;
}
