#include "forward_kinematics.hpp"

double wrapToPi(double theta) {
    double twopi = 2.0 * M_PI;
    theta = fmod(theta + M_PI, twopi);
    if (theta < 0.0) {
        theta += twopi;
    }
    return theta - M_PI;
}

/**
 * computes the forward kinematics of the robot. That is, this function computes the configuration of the delta robot
 * from measurements of the three actuated motors. It does this by guesstd::sing what the rest of the configuration is and
 * checking if that guess satisfies the holonomic constraint. Then, we apply Newton-Raphson root-finding technique to
 * update that initial guess to get closer to the root of the holonomic constraint, which, when satisfied, should be
 * the robot's configuration.
 * @param phi an estimation of the three motor angles
 * @param initial_guess a guess to seed the Newton-Raphson approximation (usually the previous known configuration.
 * @param robot_params the parameters of the delta robot (lengths of each link and radius of base + platform).
 * @param converge_params algorithm convergence parameters (such as absolute tolerance of 'x' and max iterations)
 * @return a struct containing the best estimate of the robot's configuration as well as some metrics about how quickly
 * the algorithm converged on a solution.
 */
ForwardKinematicsReturnType
forward_kinematics(Eigen::Matrix<double, 3, 1>& phi, Eigen::Matrix<double, 6, 1>& initial_guess, const RobotParameters& robot_params, const ConvergenceParameters& converge_params) {
    // extract q11, q21, and q13 from phi
    double q11 = phi[0];
    double q21 = phi[1];
    double q31 = phi[2];

    // extract robot parameters from RobotParameters object
    double l1 = robot_params.proximal_link_length();
    double l2 = robot_params.distal_link_length();
    double r_base = robot_params.base_radius();
    double r_platform = robot_params.platform_radius();

    // declare some loop parameters
    double delta_x{std::numeric_limits<double>::max()}; // infinity
    unsigned iteration{0};
    Eigen::Matrix<double, 6, 1> xk = initial_guess;
    const double sqrt3 = std::sqrt(3);

    while (iteration < converge_params.max_iterations && delta_x > converge_params.x_abstol) {
        // extract rest of the configuration parameters from the current guess
        double q12 = xk[0];
        double q13 = xk[1];
        double q22 = xk[2];
        double q23 = xk[3];
        double q32 = xk[4];
        double q33 = xk[5];

        // optimization (cos(...) once for each variable then use that local variable)

        // define the Jacobian matrix for the Newton update
        Eigen::Matrix<double, 6, 6> J {
            {l2*std::cos(q12)*std::sin(q13), l2*std::cos(q13)*std::sin(q12), (l2*(std::cos(q22)*std::sin(q23) + sqrt3*std::sin(q22)))/2, (l2*std::cos(q23)*std::sin(q22))/2, 0, 0},
            {-l2*std::sin(q12), 0, -(l2*(std::sin(q22) - sqrt3*std::cos(q22)*std::sin(q23)))/2, (sqrt3*l2*std::cos(q23)*std::sin(q22))/2, 0, 0},
            {l2*std::cos(q12)*std::cos(q13), -l2*std::sin(q12)*std::sin(q13), -l2*std::cos(q22)*std::cos(q23), l2*std::sin(q22)*std::sin(q23), 0, 0},
            {l2*std::cos(q12)*std::sin(q13), l2*std::cos(q13)*std::sin(q12), 0, 0, (l2*(std::cos(q32)*std::sin(q33) - sqrt3*std::sin(q32)))/2, (l2*std::cos(q33)*std::sin(q32))/2},
            {-l2*std::sin(q12), 0, 0, 0, -(l2*(std::sin(q32) + sqrt3*std::cos(q32)*std::sin(q33)))/2, -(sqrt3*l2*std::cos(q33)*std::sin(q32))/2},
            {l2*std::cos(q12)*std::cos(q13), -l2*std::sin(q12)*std::sin(q13), 0, 0, -l2*std::cos(q32)*std::cos(q33), l2*std::sin(q32)*std::sin(q33)}
        };

        // define the holonomic constraint vector
        Eigen::Matrix<double, 6, 1> h {
            l2*std::sin(q12)*std::sin(q13) - (sqrt3*(r_base - r_platform + l1*std::cos(q21) + l2*std::cos(q22)))/2 + (l2*std::sin(q22)*std::sin(q23))/2,
            (3*r_base)/2 - (3*r_platform)/2 + l1*std::cos(q11) + l2*std::cos(q12) + (l1*std::cos(q21))/2 + (l2*std::cos(q22))/2 + (sqrt3*l2*std::sin(q22)*std::sin(q23))/2,
            l1*std::sin(q11) - l1*std::sin(q21) + l2*std::cos(q13)*std::sin(q12) - l2*std::cos(q23)*std::sin(q22),
            (sqrt3*(r_base - r_platform + l1*std::cos(q31) + l2*std::cos(q32)))/2 + l2*std::sin(q12)*std::sin(q13) + (l2*std::sin(q32)*std::sin(q33))/2,
            (3*r_base)/2 - (3*r_platform)/2 + l1*std::cos(q11) + l2*std::cos(q12) + (l1*std::cos(q31))/2 + (l2*std::cos(q32))/2 - (sqrt3*l2*std::sin(q32)*std::sin(q33))/2,
            l1*std::sin(q11) - l1*std::sin(q31) + l2*std::cos(q13)*std::sin(q12) - l2*std::cos(q33)*std::sin(q32)
        };

        // solve linear equation J ? = b <--> ? = J \ b
        // -> colPivHouseholderQr
        // -> completeOrthogonalDecomposition
        Eigen::Matrix<double, 6, 1> sol = J.fullPivLu().solve(h);

        // update the guess ustd::sing Newton-Raphson update xk = x0 - J\h
        Eigen::Matrix<double, 6, 1> xk_plus_1 = xk - sol;

        // update loop variables
        delta_x  = (sol).array().abs().maxCoeff(); // l-infinity norm (not sure what norm to use here?)
        xk = xk_plus_1;
        iteration += 1;
    }

    ForwardKinematicsReturnType ret;
    ret.robot_state = Eigen::Matrix<double, 9, 1> {
        wrapToPi(q11),
        wrapToPi(xk[0]),
        wrapToPi(xk[1]),
        wrapToPi(q21),
        wrapToPi(xk[2]),
        wrapToPi(xk[3]),
        wrapToPi(q31),
        wrapToPi(xk[4]),
        wrapToPi(xk[5])
    };

#ifndef NDEBUG
    ret.x_abserr = delta_x;
    ret.number_of_iterations = iteration;
#endif // NDEBUG
    return ret;
}