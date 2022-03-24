#ifndef DELTA_ROBOT_STATE_HPP
#define DELTA_ROBOT_STATE_HPP

#include <Eigen/Eigen>

class RobotState {
private:
    Eigen::VectorXd state_;

public:
    Eigen::VectorXd state();
    void update();
};

#endif //DELTA_ROBOT_STATE_HPP
