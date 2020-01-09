#include "obstacles_trajectory_predictor/kf.h"

KalmanFilter::KalmanFilter(void)
{
    sigma_a = 0.1;
}

Eigen::Matrix4d KalmanFilter::get_f(double dt)
{
    Eigen::Matrix4d f;
    f << 1.0, 0.0,  dt, 0.0,
         0.0, 1.0, 0.0,  dt,
         0.0, 0.0, 1.0, 0.0,
         0.0, 0.0, 0.0, 1.0;
    return f;
}

Eigen::Matrix4d KalmanFilter::get_q(double dt)
{
    Eigen::Matrix<double, 4, 2> g;
    g << dt * dt / 2.0,           0.0,
                   0.0, dt * dt / 2.0,
                    dt,           0.0,
                   0.0,            dt;
    Eigen::Matrix4d q = sigma_a * sigma_a * g * g.transpose();
    return q;
}
