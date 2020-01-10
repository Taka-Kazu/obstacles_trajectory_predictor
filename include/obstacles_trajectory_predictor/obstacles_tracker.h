#ifndef __OBSTACLES_TRACKER_H
#define __OBSTACLES_TRACKER_H

#include <boost/optional.hpp>

#include "obstacles_trajectory_predictor/obstacle.h"
#include "obstacles_trajectory_predictor/social_force_model.h"

class ObstaclesTracker
{
public:
    ObstaclesTracker(void);

    void set_obstacles_position(const std::vector<Eigen::Vector2d>&);
    std::vector<Eigen::Vector2d> get_velocities(void);
    std::vector<Eigen::Vector2d> get_positions(void);
    typedef int obstacle_index_t;

private:
    bool associate_obstacles(const std::vector<Eigen::Vector2d>&, std::vector<int>&);
    double get_distance(const Obstacle&, const Eigen::Vector2d&);
    int get_id_from_index(int);
    int get_new_id(void);
    bool solve_hungarian_method(Eigen::MatrixXi&, std::vector<int>&);
    void update_tracking(const std::vector<Eigen::Vector2d>&, const std::vector<int>&);
    std::vector<Obstacle> get_obstacles(const std::map<int, Obstacle>&);

    double SAME_OBSTACLE_THRESHOLD;
    double ERASE_LIKELIHOOD_THREHSOLD;
    double NOT_OBSERVED_TIME_THRESHOLD;
    double DEFAULT_LIFE_TIME;
    std::map<int, Obstacle> obstacles;
};

#endif// __OBSTACLES_TRACKER_H
