#include "obstacles_trajectory_predictor/obstacles_trajectory_predictor.h"

ObstaclesTrajectoryPredictor::ObstaclesTrajectoryPredictor(void)
{

}

void ObstaclesTrajectoryPredictor::set_obstacles_position(const std::vector<Eigen::Vector2d>& obstacles_position)
{
    tracker.set_obstacles_position(obstacles_position);
}

void ObstaclesTrajectoryPredictor::set_static_obstacles_position(const std::vector<Eigen::Vector2d>& static_obstacles_position)
{
    tracker.set_static_obstacles_position(static_obstacles_position);
}

std::vector<Eigen::Vector2d> ObstaclesTrajectoryPredictor::get_velocities(void)
{
    return tracker.get_velocities();
}

std::vector<Eigen::Vector2d> ObstaclesTrajectoryPredictor::get_positions(void)
{
    return tracker.get_positions();
}

std::vector<int> ObstaclesTrajectoryPredictor::get_ids(void)
{
    return tracker.get_ids();
}

std::vector<std::vector<Obstacle> > ObstaclesTrajectoryPredictor::simulate(int simulation_step)
{
    auto obstacles = tracker.get_obstacles();
    int obstacles_num = obstacles.size();
    std::cout << "simulate " << obstacles_num << " obstacles for " << simulation_step << " steps" << std::endl;
    std::vector<std::vector<Obstacle> > obstacles_predicted_states(obstacles_num, std::vector<Obstacle>(simulation_step + 1));
    for(int n=0;n<obstacles_num;n++){
        obstacles_predicted_states[n][0] = obstacles[n];
    }
    for(int i=1;i<simulation_step+1;i++){
        // std::cout << "simulation step: " << i << std::endl;
        obstacles = tracker.simulate_one_step(obstacles);
        for(int n=0;n<obstacles_num;n++){
            // std::cout << "obstacle: " << n << std::endl;
            obstacles_predicted_states[n][i] = obstacles[n];
        }
    }
    return obstacles_predicted_states;
}
