/// Standard Library Includes
#include <tuple>
#include <filesystem>

/// External Library Includes
#include <gtest/gtest.h>
#include <Eigen/Dense>

/// Internal Library Includes
#include "forward_kinematics.hpp"
#include "robot_parameters.hpp"
#include "defines.hpp"

class ForwardKinTester : public ::testing::Test {
protected:


protected:
    virtual void SetUp() {

    }
};

TEST_F(ForwardKinTester, Callable) {
    Eigen::Matrix<double, 3, 1> phi {0.0, 0.0, 0.0};
    Eigen::Matrix<double, 9, 1> initial_guess{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    RobotParameters robot_params(1.0, 1.0, 1.0, 1.0);
    ConvergenceParameters convergence_params(1e-6, 1e-12, 1e-6, 1e-12, 100);

    Eigen::Matrix<double, 9, 1> state = forward_kinematics(phi, initial_guess, robot_params, convergence_params);

    std::cout << state << std::endl;

    for (int i = 0; i < state.size(); i++) {
        EXPECT_FLOAT_EQ(state[i], initial_guess[i]);
    }

}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

