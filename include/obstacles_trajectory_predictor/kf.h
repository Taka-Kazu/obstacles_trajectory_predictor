#ifndef __KF_H
#define __KF_H

#include <iostream>

#include <Eigen/Dense>

class KalmanFilter
{
public:
    KalmanFilter(void);

    Eigen::Matrix4d get_f(double);// transition model
    Eigen::Matrix4d get_q(double);// transition noise

    double sigma_a;
private:
};

#endif// __KF_H
