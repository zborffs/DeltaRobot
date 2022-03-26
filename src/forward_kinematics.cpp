#include "forward_kinematics.hpp"

/**
 *
 * @param phi
 * @param initial_guess
 * @param robot_params
 * @param converge_params
 * @return
 */
Eigen::Matrix<double, 9, 1>
forward_kinematics(Eigen::Matrix<double, 3, 1>& phi, Eigen::Matrix<double, 9, 1>& initial_guess, const RobotParameters& robot_params, ConvergenceParameters& converge_params) {
    return initial_guess;
}