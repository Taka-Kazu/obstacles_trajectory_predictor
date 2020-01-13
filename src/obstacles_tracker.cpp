#include "obstacles_trajectory_predictor/obstacles_tracker.h"

ObstaclesTracker::ObstaclesTracker(void)
:SAME_OBSTACLE_THRESHOLD(0.8), ERASE_LIKELIHOOD_THREHSOLD(100)
, NOT_OBSERVED_TIME_THRESHOLD(1.0), DEFAULT_LIFE_TIME(1.0)
, DT(0.1), MOVING_THRESHOLD(0.3), MAX_SOCIAL_FORCE_THREHOLD(10)
, VERBOSE(false)
{
    obstacles.clear();
}

void ObstaclesTracker::set_obstacles_position(const std::vector<Eigen::Vector2d>& observed_obstacles_)
{
    // observation
    std::cout << "tracking " << obstacles.size() << " obstacles" << std::endl;
    std::cout << "observed " << observed_obstacles_.size() << " obstacles" << std::endl;

    std::vector<int> association_candidates;
    std::vector<Eigen::Vector2d> observed_obstacles(observed_obstacles_);
    bool is_suceeded = associate_obstacles(observed_obstacles, association_candidates);
    if(!is_suceeded){
        std::cout << "\033[31m=====================\nfailed to associate obstacles\n=====================\033[0m" << std::endl;
        observed_obstacles.clear();
        associate_obstacles(observed_obstacles, association_candidates);
    }

    update_tracking(observed_obstacles, association_candidates);

    sfm.set_observed_agents_state(get_obstacles(obstacles));

    std::cout << "--- predict ---" << std::endl;
    auto it = obstacles.begin();
    misrecognition_ids.clear();
    while(it != obstacles.end()){
        if(VERBOSE){
            std::cout << "-\nobstacle id: " << it->first << std::endl;
            std::cout << "before prediction" << std::endl;
            std::cout << it->second.get_position().transpose() << std::endl;
            std::cout << it->second.get_velocity().transpose() << std::endl;
        }
        Eigen::Vector2d sf = sfm.get_social_force(std::distance(obstacles.begin(), it));
        if(VERBOSE){
            std::cout << "force: " << sf.transpose() << std::endl;
        }
        if(MAX_SOCIAL_FORCE_THREHOLD < sf.norm()){
            if((it->second.age < it->second.lifetime * 5)
               || (it->second.calculate_likelihood() < ERASE_LIKELIHOOD_THREHSOLD)){
                if(VERBOSE){
                    std::cout << "maybe misrecognition" << std::endl;
                }
                misrecognition_ids.emplace_back(it->first);
            }
        }
        it->second.predict(sf);
        if(VERBOSE){
            std::cout << "after prediction" << std::endl;
            std::cout << it->second.get_position().transpose() << std::endl;
            std::cout << it->second.get_velocity().transpose() << std::endl;
            std::cout << "not observed: " << it->second.not_observed_time << std::endl;
            std::cout << "age: " << it->second.age << std::endl;
            std::cout << "likelihood: " << it->second.calculate_likelihood() << std::endl;
        }
        if(it->second.age < 0.0){
            if(VERBOSE){
                std::cout << "\033[31mobstacle " << it->first << " was erased\033[0m" << std::endl;
            }
            it = obstacles.erase(it);
            continue;
        }
        if((it->second.calculate_likelihood() > ERASE_LIKELIHOOD_THREHSOLD) && (it->second.not_observed_time < NOT_OBSERVED_TIME_THRESHOLD)){
            ++it;
        }else{
            if(it->second.lifetime > it->second.age){
                ++it;
                if(VERBOSE){
                    std::cout << "\033[33mthe likelihood of this obstacle is small but is not old enough to be erased\033[0m" << std::endl;
                }
            }else{
                if(VERBOSE){
                    std::cout << "\033[31mobstacle " << it->first << " was erased\033[0m" << std::endl;
                }
                it = obstacles.erase(it);
            }
        }
    }
}

void ObstaclesTracker::set_static_obstacles_position(const std::vector<Eigen::Vector2d>& static_obstacles_position)
{
    static_obstacles = static_obstacles_position;
}

void ObstaclesTracker::setup_simulation(const std::vector<Obstacle>& agents)
{
    sfm.set_observed_agents_state(agents);
}

bool ObstaclesTracker::associate_obstacles(const std::vector<Eigen::Vector2d>& observed_obstacles, std::vector<int>& association_candidates)
{
    std::cout << "--- associate obstacles ---" << std::endl;
    if(VERBOSE){
        std::cout << "tracking obstacles:" << std::endl;
        for(const auto& to : obstacles){
            std::cout << to.second.x.segment(0, 2).transpose() << std::endl;
        }
        std::cout << "observed obstacles:" << std::endl;
        for(const auto& oo : observed_obstacles){
            std::cout << oo.transpose() << std::endl;
        }
    }
    int cluster_num = obstacles.size();
    int observed_obstacles_num = observed_obstacles.size();

    int matrix_size = cluster_num + observed_obstacles_num;

    Eigen::MatrixXi association_matrix(matrix_size, matrix_size);
    for(int i=0;i<matrix_size;i++){
        for(int j=0;j<matrix_size;j++){
            if(i < cluster_num && j < observed_obstacles_num){
                double distance = get_distance(obstacles[get_id_from_index(i)], observed_obstacles[j]);
                // std::cout << i << ", " << j << ": " << distance << std::endl;
                association_matrix(i, j) = distance;
                // std::cout << "i, j = " << i << ", " << j << " : " << distance << std::endl;
            }else if(i < cluster_num && !(j < observed_obstacles_num)){
                association_matrix(i, j) = SAME_OBSTACLE_THRESHOLD * 100;
            }else if(!(i < cluster_num) && j < observed_obstacles_num){
                association_matrix(i, j) = SAME_OBSTACLE_THRESHOLD * 100;
            }else{
                association_matrix(i, j) = SAME_OBSTACLE_THRESHOLD * 100;
            }
        }
    }
    if(VERBOSE){
        std::cout << "association matrix:\n" << association_matrix << std::endl;
    }
    return solve_hungarian_method(association_matrix, association_candidates);
}

double ObstaclesTracker::get_distance(const Obstacle& obstacle, const Eigen::Vector2d& position)
{
    // std::cout << "--- get distance ---" << std::endl;
    // std::cout << obstacle.x.segment(0, 2).transpose() << " vs " << position.transpose() << std::endl;
    double likelihood = (obstacle.x.segment(0, 2) - position).norm() * 100;
    return likelihood;
}

int ObstaclesTracker::get_id_from_index(int index)
{
    if(index < 0){
        return -1;
    }
    int index_ = 0;
    for(auto it=obstacles.begin();it!=obstacles.end();++it){
        if(index_ == index){
            return it->first;
        }
        index_++;
    }
    return -1;
}

int ObstaclesTracker::get_new_id(void)
{
    int new_id = 0;
    int obstacles_num = obstacles.size();
    for(int i=0;i<obstacles_num;i++){
        bool used_id_flag = false;
        for(auto it=obstacles.begin();it!=obstacles.end();++it){
            if(new_id == it->first){
                used_id_flag = true;
                break;
            }
        }
        if(!used_id_flag){
            return new_id;
        }
        new_id++;
    }
    return new_id;
}

bool ObstaclesTracker::solve_hungarian_method(Eigen::MatrixXi& matrix, std::vector<int>& association_candidates)
{
    // reference: http://www.prefield.com/algorithm/math/hungarian.html
    const double inf = 1e6;
    int n = matrix.rows(), p, q;
    std::vector<int> fx(n, inf), fy(n, 0);
    std::vector<int> x(n, -1), y(n, -1);
    for(int i = 0;i < n;++i){
        for(int j = 0;j < n;++j){
            fx[i] = std::min(fx[i], matrix(i, j));
        }
    }
    const int MAX_ITERATION = 100;
    int count = 0;
    for(int i = 0;i < n;){
        // std::cout << "count: " << count << std::endl;
        if(count > MAX_ITERATION){
            std::cout << "\033[31mafter " << MAX_ITERATION << " times loop,  break!!!!!\033[0m" << std::endl;
            return false;
        }
        std::vector<int> t(n, -1), s(n+1, i);
        for(p = q = 0;p <= q && x[i] < 0;++p){
            for(int k = s[p], j = 0;j < n && x[i] < 0;++j){
                if(fx[k] + fy[j] == matrix(k, j) && t[j] < 0){
                    s[++q] = y[j];
                    t[j] = k;
                    if(s[q] < 0){
                        for(p = j;p >= 0;j = p){
                            y[j] = k = t[j];
                            p = x[k];
                            x[k] = j;
                        }
                    }
                }
            }
        }
        if(x[i] < 0){
            int d = inf;
            for(int k = 0;k <= q;++k){
                for(int j = 0;j < n;++j){
                    if(t[j] < 0){
                        d = std::max(d, fx[s[k]] + fy[j] - matrix(s[k], j));
                    }
                }
                for(int j = 0;j < n;++j){
                    fy[j] += (t[j] < 0 ? 0 : d);
                }
                for(int k = 0;k <= q;++k){
                    fx[s[k]] -= d;
                }
            }
        }else{
            ++i;
        }
        count++;
    }
    // std::cout << "candidates: " << std::endl;
    association_candidates.resize(n);
    for(int i = 0;i < n;++i){
        association_candidates[i] = get_id_from_index(y[i]);
        // std::cout << i << ": " << candidates[i] << std::endl;
        // std::cout << "index: " << y[i] << std::endl;
    }
    return true;
}

void ObstaclesTracker::update_tracking(const std::vector<Eigen::Vector2d>& observed_obstacles, const std::vector<int>& association_candidates)
{
    std::cout << "--- update tracking ---" << std::endl;
    for(auto it=observed_obstacles.begin();it!=observed_obstacles.end();++it){
        int id = association_candidates[it - observed_obstacles.begin()];
        auto obstacle_it = obstacles.find(id);
        if(obstacle_it != obstacles.end()){
            // this obstacle has already been tracked
            obstacle_it->second.update(*it);
        }else{
            // new obstacle
            Obstacle obstacle(*it);
            int new_id = get_new_id();
            obstacles[new_id] = obstacle;
            std::cout << "new obstacle (id:" << new_id << ") was added" << std::endl;
        }
    }
}

std::vector<Obstacle> ObstaclesTracker::get_obstacles(const ObstaclesWithID& obs)
{
    std::vector<Obstacle> obstacles_;
    for(const auto o : obs){
        obstacles_.emplace_back(o.second);
    }
    return obstacles_;
}

std::vector<Obstacle> ObstaclesTracker::get_obstacles(void)
{
    return get_obstacles(obstacles);
}

std::vector<Obstacle> ObstaclesTracker::get_moving_obstacles(void)
{
    std::cout << "obstacles: " << obstacles.size() << std::endl;
    std::vector<Obstacle> obstacles_;
    for(const auto o : obstacles){
        if(o.second.get_velocity().norm() > MOVING_THRESHOLD){
            if(std::find(misrecognition_ids.begin(), misrecognition_ids.end(), o.first) == misrecognition_ids.end()){
                obstacles_.emplace_back(o.second);
            }
        }
    }
    std::cout << "moving obstacles: " << obstacles_.size() << std::endl;
    return obstacles_;
}

std::vector<Eigen::Vector2d> ObstaclesTracker::get_velocities(void)
{
    std::vector<Eigen::Vector2d> velocities;
    for(auto it=obstacles.begin();it!=obstacles.end();++it){
        velocities.push_back(it->second.get_velocity());
    }
    return velocities;
}

std::vector<Eigen::Vector2d> ObstaclesTracker::get_positions(void)
{
    std::vector<Eigen::Vector2d> positions;
    for(auto it=obstacles.begin();it!=obstacles.end();++it){
        positions.push_back(it->second.get_position());
    }
    return positions;
}


std::vector<int> ObstaclesTracker::get_ids(void)
{
    std::vector<int> ids;
    for(auto it=obstacles.begin();it!=obstacles.end();++it){
        ids.push_back(it->first);
    }
    return ids;
}

std::vector<Obstacle> ObstaclesTracker::simulate_one_step(const std::vector<Obstacle>& obstacles_)
{
    std::vector<Obstacle> local_obstacles = obstacles_;
    sfm.set_agents_state(local_obstacles);
    sfm.set_objects(static_obstacles);

    auto it = local_obstacles.begin();
    while(it != local_obstacles.end()){
        // std::cout << "before prediction" << std::endl;
        // std::cout << it->get_position().transpose() << std::endl;
        // std::cout << it->get_velocity().transpose() << std::endl;
        Eigen::Vector2d sf = sfm.get_social_force(std::distance(local_obstacles.begin(), it));
        it->predict(sf, DT);
        // std::cout << "after prediction" << std::endl;
        // std::cout << it->get_position().transpose() << std::endl;
        // std::cout << it->get_velocity().transpose() << std::endl;
        ++it;
    }
    return local_obstacles;
}

void ObstaclesTracker::set_verbose_output(bool verbose)
{
    VERBOSE = verbose;
}
