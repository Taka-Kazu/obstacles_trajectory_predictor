#ifndef __SOCIAL_FORCE_MODEL_H
#define __SOCIAL_FORCE_MODEL_H

// ref: http://srl.informatik.uni-freiburg.de/papers/luberICRA10.pdf

#include <ros/ros.h>

#include <Eigen/Dense>

#include "obstacles_trajectory_predictor/obstacle.h"

class SocialForceModel
{
public:
    SocialForceModel(void);

    void set_agents_state(const std::vector<Obstacle>&);
    void set_observed_agents_state(const std::vector<Obstacle>&);
    void set_objects(const std::vector<Eigen::Vector2d>&);
    Eigen::Vector2d get_virtual_goal(const Obstacle&);
    Eigen::Vector2d get_intended_direction(size_t);
    Eigen::Vector2d get_intended_velocity_vector(size_t);
    Eigen::Vector2d get_social_force(size_t);
    Eigen::Vector2d get_personal_motivation_force(size_t);
    Eigen::Vector2d get_interaction_force(size_t);
    Eigen::Vector2d get_interaction_force_agents(size_t);
    Eigen::Vector2d get_interaction_force_objects(size_t);

private:
    double DT;
    double LAMBDA;
    double DELTA_G;// goal ahead time[s]
    double TAU;// relaxation time[s]
    double MAGNITUDE_AGENT;// [m]
    double FORCE_RANGE_AGENT;// [N]
    double MAGNITUDE_OBJECT;// [m]
    double FORCE_RANGE_OBJECT;// [N]

    std::vector<Obstacle> agents;
    std::vector<Eigen::Vector2d> virtual_goals;
    std::vector<Eigen::Vector2d> objects;
};

#endif// __SOCIAL_FORCE_MODEL_H
