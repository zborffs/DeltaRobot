#ifndef DELTA_ENVIRONMENT_STATE_HPP
#define DELTA_ENVIRONMENT_STATE_HPP

#include <Eigen/Eigen>

class EnvironmentState {
private:
    Eigen::VectorXd state_;

public:
    Eigen::VectorXd state();
    void update();
};

#endif //DELTA_ENVIRONMENT_STATE_HPP
