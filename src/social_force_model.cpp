#include "obstacles_trajectory_predictor/social_force_model.h"

SocialForceModel::SocialForceModel(void)
{
    DT = 0.1;
    LAMBDA = 0.5;
    DELTA_G = 60 * DT;
    TAU = 0.5;
    MAGNITUDE_AGENT = 70;
    FORCE_RANGE_AGENT = 0.4;
    MAGNITUDE_OBJECT = 100;
    FORCE_RANGE_OBJECT = 0.01;
}

void SocialForceModel::set_agents_states(const std::vector<Obstacle>& agents_)
{
    agents.clear();
    agents = agents_;
}

void SocialForceModel::set_objects(const std::vector<Eigen::Vector2d>& objects_)
{
    objects.clear();
    objects = objects_;
}

Eigen::Vector2d SocialForceModel::get_virtual_goal(Obstacle& agent)
{
    return agent.get_position() + agent.get_velocity() * DELTA_G;
}

Eigen::Vector2d SocialForceModel::get_intended_direction(Obstacle& agent)
{
    Eigen::Vector2d g = get_virtual_goal(agent);
    Eigen::Vector2d x = agent.get_position();
    return (g - x).normalized();
}

Eigen::Vector2d SocialForceModel::get_intended_velocity_vector(Obstacle& agent)
{
    Eigen::Vector2d g = get_virtual_goal(agent);
    Eigen::Vector2d x = agent.get_position();
    return (g - x) / DELTA_G;
}

Eigen::Vector2d SocialForceModel::get_social_force(size_t index)
{
    size_t n_agents = agents.size();
    if(n_agents <= index){
        std::cout << "error: index out of range, index is " << index << ", size of agents is " << n_agents << std::endl;
    }
    Obstacle agent = agents[index];
    return get_personal_motivation_force(agent) + get_interaction_force(index);
}

Eigen::Vector2d SocialForceModel::get_personal_motivation_force(Obstacle& agent)
{
    return agent.mass * (get_intended_velocity_vector(agent) - agent.get_velocity()) / TAU;
}

Eigen::Vector2d SocialForceModel::get_interaction_force(size_t index)
{
    Eigen::Vector2d force = get_interaction_force_agents(index) + get_interaction_force_objects(index);
    return force;
}

Eigen::Vector2d SocialForceModel::get_interaction_force_agents(size_t index)
{
    size_t n_agents = agents.size();
    Obstacle agent = agents[index];
    Eigen::Vector2d intended_direction = get_intended_direction(agent);
    Eigen::Vector2d force = Eigen::Vector2d::Zero();
    for(size_t i=0;i<n_agents;i++){
        if(index == i){
            // skip itself
            continue;
        }
        double r = agent.radius + agents[i].radius;
        double d = (agent.get_position() - agents[i].get_position()).norm();
        Eigen::Vector2d n = (agent.get_position() - agents[i].get_position()).normalized();
        force += MAGNITUDE_AGENT * exp((r - d) / FORCE_RANGE_AGENT) * n * (LAMBDA + (1 - LAMBDA) * (1 - n.dot(intended_direction)) * 0.5);
    }
    return force;
}

Eigen::Vector2d SocialForceModel::get_interaction_force_objects(size_t index)
{
    // unimplemented
    size_t n_objects = objects.size();
    Obstacle agent = agents[index];
    Eigen::Vector2d intended_direction = get_intended_direction(agent);
    Eigen::Vector2d force = Eigen::Vector2d::Zero();
    for(size_t i=0;i<n_objects;i++){
        double d = (agent.get_position() - objects[i]).norm();
        Eigen::Vector2d n = (agent.get_position() - objects[i]).normalized();
        force += MAGNITUDE_OBJECT * exp((agent.radius - d) / FORCE_RANGE_OBJECT) * n * (LAMBDA + (1 - LAMBDA) * (1 - n.dot(intended_direction)) * 0.5);
    }
    return force;
}
