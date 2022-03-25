/**
 *
 * This file contains a function definition for the forward kinematics of the delta robot and a definition of the struct
 * returned by the forward kinematics function. The struct isn't __strictly__ necessary, but it doesn't do much harm.
 * Basically, it returns several performance metrics of the function in addition to the desired return value.
 *
 * The forward kinematics function essentially receives known actuator joint angles and outputs the full configuration
 * of the robot. In this particular case, it goes about solving that problem by numerically computing the state using
 * Newton-Raphson.
 *
 */
#ifndef DELTA_DELTA_UTILITIES_HPP
#define DELTA_DELTA_UTILITIES_HPP

/// internal includes
#include "robot_parameters.hpp"
#include "defines.hpp"

/// third party includes
#include <Eigen/Eigen>

struct ForwardKinematicsReturnType {
    Eigen::VectorXd robot_state; // best guess of the state from the motor encodings
#ifndef NDEBUG
    // if we are in debug mode, measure how fast we converge on a solution by measuring these things. But to save on
    // compute, don't measure or save these if we are in a release.

    unsigned int number_of_iterations; // number of iterations it took to converge
    double x_abserr; // absolute error between successive guesses
    double x_relerr; // relative error between successive guesses
    double f_x_abserr; // absolute error between successive guesses
    double f_x_relerr; // relative error between successive guesses
    double elapsed_time_in_us; // how long it took to get an answer in microseconds (goal is < 1 us on Jetson and Mac)

#endif // NDEBUG
};

ForwardKinematicsReturnType
forward_kin(
    Eigen::VectorXd& phi, // what we directly measure from the encoder (i.e. angles of actuated joints)
    Eigen::VectorXd& initial_guess, // our guess of what the state might be (maybe seed with previous state)
    const RobotParameters& robot_params, // the parameters of the robot (know this a priori)
    ConvergenceParameters& converge_params); // configurable parameters chosen at run-time (tolerances and so forth)

#endif //DELTA_DELTA_UTILITIES_HPP
