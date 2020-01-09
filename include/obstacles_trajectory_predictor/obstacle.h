#ifndef __OBSTACLE_H
#define __OBSTACLE_H

#include <ros/ros.h>

#include <Eigen/Dense>

#include "obstacles_trajectory_predictor/kf.h"

class Obstacle
{
public:
    Obstacle(void);
    Obstacle(const Obstacle&);
    Obstacle(const Eigen::Vector2d&);

    void update(const Eigen::Vector2d&);
    void predict(void);
    void predict(double);
    Eigen::Vector2d get_position(void);
    double calculate_likelihood(void);

    Eigen::Vector2d position;
    Eigen::Vector4d x;
    Eigen::Matrix4d p;
    Eigen::Matrix<double, 2, 4> h;
    double likelihood;
    double lifetime;
    double age;
    double not_observed_time;
private:
    void initialize(void);

    Eigen::Matrix2d r;
    KalmanFilter kf;
    double last_time;
};

#endif// __OBSTACLE_H
