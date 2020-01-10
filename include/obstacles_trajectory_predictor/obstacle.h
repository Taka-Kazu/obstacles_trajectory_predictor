#ifndef __OBSTACLE_H
#define __OBSTACLE_H

#include <ros/ros.h>

#include <Eigen/Dense>

class Obstacle
{
public:
    Obstacle(void);
    Obstacle(const Obstacle&);
    Obstacle(const Eigen::Vector2d&);

    Eigen::Vector4d get_next_state(const Eigen::Vector4d&, const Eigen::Vector2d&, double);
    Eigen::Matrix4d get_jacobian_f(double);
    Eigen::Matrix4d get_state_transition_noise_matrix(double);
    void update(const Eigen::Vector2d&);
    void predict(const Eigen::Vector2d&, double);
    void predict(const Eigen::Vector2d&);
    Eigen::Vector2d get_position(void);
    Eigen::Vector2d get_velocity(void);
    double calculate_likelihood(void);

    Eigen::Vector4d x;// x, y, \dot{x}, \dot{y}
    Eigen::Matrix4d p;
    Eigen::Matrix<double, 2, 4> h;
    double likelihood;
    double lifetime;
    double age;
    double not_observed_time;
    double mass;
    double radius;

private:
    void initialize(void);

    Eigen::Matrix2d r;
    double last_time;
};

#endif// __OBSTACLE_H
