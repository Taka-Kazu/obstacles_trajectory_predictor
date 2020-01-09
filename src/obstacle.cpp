#include "obstacles_trajectory_predictor/obstacle.h"

Obstacle::Obstacle(void)
{
    x = Eigen::Vector4d::Zero();

    initialize();
}

Obstacle::Obstacle(const Obstacle& obstacle)
{
    x = obstacle.x;
    p = obstacle.p;
    r = obstacle.r;
    h = obstacle.h;
    likelihood = obstacle.likelihood;
    last_time = obstacle.last_time;
    lifetime = obstacle.lifetime;
    age = obstacle.age;
    not_observed_time = obstacle.not_observed_time;
}

Obstacle::Obstacle(const Eigen::Vector2d& position)
{
    x << position(0), position(1), 0.0, 0.0;

    initialize();
}

void Obstacle::initialize(void)
{
    p << 1e0, 0.0, 0.0, 0.0,
         0.0, 1e0, 0.0, 0.0,
         0.0, 0.0, 1e0, 0.0,
         0.0, 0.0, 0.0, 1e0;

    h << 1.0, 0.0, 0.0, 0.0,
         0.0, 1.0, 0.0, 0.0;

    r << 1e-4,  0.0,
          0.0, 1e-4;

    last_time = ros::Time::now().toSec();
    likelihood = 1.0;
    lifetime = 1;
    age = 0;
    not_observed_time = 0;
    MASS = 60;
}

Eigen::Vector4d Obstacle::get_next_state(const Eigen::Vector4d& x0, const Eigen::Vector2d& force, double dt)
{
    Eigen::Vector4d x1 = Eigen::Vector4d::Zero();
    Eigen::Vector2d acc = force / MASS;
    x1.segment(0, 2) = x0.segment(0, 2) + x0.segment(2, 2) * dt + 0.5 * acc * dt * dt;
    x1.segment(2, 2) = x0.segment(2, 2) + acc * dt;
    return x1;
}

Eigen::Matrix4d Obstacle::get_jacobian_f(double dt)
{
    Eigen::Matrix4d jf;
    jf << 1.0, 0.0,  dt, 0.0,
          0.0, 1.0, 0.0,  dt,
          0.0, 0.0, 1.0, 0.0,
          0.0, 0.0, 0.0, 1.0;
    return jf;
}

Eigen::Matrix4d Obstacle::get_state_transition_noise_matrix(double dt)
{
    double sigma_a = 0.1;
    Eigen::Matrix<double, 4, 2> g;
    g << dt * dt / 2.0,           0.0,
                   0.0, dt * dt / 2.0,
                    dt,           0.0,
                   0.0,            dt;
    Eigen::Matrix4d q = sigma_a * sigma_a * g * g.transpose();
    return q;
}

Eigen::Vector2d Obstacle::get_position(void)
{
    return x.segment(0, 2);
}

void Obstacle::update(const Eigen::Vector2d& z)
{
    // std::cout << "update" << std::endl;
    // std::cout << "Z:\n" << z << std::endl;
    Eigen::Vector2d e = z - h * x;
    Eigen::Matrix2d s = h * p * h.transpose() + r;
    Eigen::Matrix<double, 4, 2> k = p * h.transpose() * s.inverse();
    // std::cout << "K:\n" << k << std::endl;
    x = x + k * e;
    // std::cout << "X:\n" << x << std::endl;
    p = (Eigen::Matrix4d::Identity() - k * h) * p;
    // std::cout << "P:\n" << p << std::endl;

    not_observed_time = 0;
}

void Obstacle::predict(void)
{
    // std::cout << "predict" << std::endl;
    double current_time = ros::Time::now().toSec();
    double dt = current_time - last_time;
    last_time = current_time;
    predict(dt);
}

void Obstacle::predict(double dt)
{
    // std::cout << "predict" << std::endl;
    age += dt;
    not_observed_time += dt;
    // std::cout << "age: " << age << std::endl;
    // std::cout << "not_observed_time: " << not_observed_time << std::endl;

    Eigen::Vector2d force = Eigen::Vector2d::Zero();
    x = get_next_state(x, force, dt);
    Eigen::Matrix4d jf = get_jacobian_f(dt);
    // std::cout << "X:\n" << x << std::endl;;
    Eigen::Matrix4d q = get_state_transition_noise_matrix(dt);
    p = jf * p * jf.transpose() + q;
    // std::cout << "P:\n" << p << std::endl;;
}

void Obstacle::predict(const Eigen::Vector2d& force, double dt)
{
    // std::cout << "predict" << std::endl;
    age += dt;
    not_observed_time += dt;
    // std::cout << "age: " << age << std::endl;
    // std::cout << "not_observed_time: " << not_observed_time << std::endl;

    x = get_next_state(x, force, dt);
    Eigen::Matrix4d jf = get_jacobian_f(dt);
    // std::cout << "X:\n" << x << std::endl;;
    Eigen::Matrix4d q = get_state_transition_noise_matrix(dt);
    p = jf * p * jf.transpose() + q;
    // std::cout << "P:\n" << p << std::endl;;
}

double Obstacle::calculate_likelihood(void)
{
    Eigen::Matrix2d m = p.block<2, 2>(0, 0);// up left 2x2
    Eigen::EigenSolver<Eigen::Matrix2d> es(m);
    if(!es.info()){
        Eigen::Vector2d e_values = es.eigenvalues().real();
        const double CHI2 = 9.21034;// chi-square, 99%
        double a, b;// ellipse parameter
        if(e_values(0) > e_values(1)){
            a = sqrt(CHI2 * e_values(0));
            b = sqrt(CHI2 * e_values(1));
        }else{
            a = sqrt(CHI2 * e_values(1));
            b = sqrt(CHI2 * e_values(0));
        }
        if(a * b > 1e-5){
            likelihood = 1.0 / (a * b);
        }else{
            likelihood = 1e3;
        }
    }else{
        std::cout << "Eigen solver error: " << es.info() << std::endl;
    }
    return likelihood;
}
