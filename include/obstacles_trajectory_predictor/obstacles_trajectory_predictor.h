#ifndef __OBSTACLES_TRAJECTORY_PREDICTOR_H
#define __OBSTACLES_TRAJECTORY_PREDICTOR_H

#include "obstacles_trajectory_predictor/obstacles_tracker.h"

class ObstaclesTrajectoryPredictor
{
public:
    ObstaclesTrajectoryPredictor(void);

    void set_obstacles_position(const std::vector<Eigen::Vector2d>&);
    std::vector<Eigen::Vector2d> get_velocities(void);
    std::vector<Eigen::Vector2d> get_positions(void);
    std::vector<int> get_ids(void);
    std::vector<std::vector<Obstacle> >  simulate(int);

private:
    ObstaclesTracker tracker;
};

#endif// __OBSTACLES_TRAJECTORY_PREDICTOR_H
