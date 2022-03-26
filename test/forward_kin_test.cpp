/// Standard Library Includes
#include <tuple>
#include <filesystem>
#include <cmath>
#include <fstream>

/// External Library Includes
#include <gtest/gtest.h>
#include <Eigen/Dense>

/// Internal Library Includes
#include "forward_kinematics.hpp"
#include "robot_parameters.hpp"
#include "defines.hpp"
#include "stopwatch.hpp"

std::vector<std::string> split(const std::string& str, const char delim) {
    std::vector<std::string> ret;
    std::string::const_iterator first  = str.cbegin();
    std::string::const_iterator second = str.cbegin();

    for(; second <= str.end(); ++second) {
        if(*(second) == delim || second == str.end()) {
            if(second != first) {
                ret.emplace_back(first, second);
            }

            first = second + 1;
        }
    }

    return ret;
}

std::vector<std::string> read_file(const std::string& file_name) {
    std::vector<std::string> lines;

    try {
        std::fstream in(file_name);
        in.exceptions(std::ifstream::badbit);
        if(!in.is_open()) {
            std::string err("Failed to open file: ");
            err += file_name;
            throw std::runtime_error(err);
        }

        std::string line;
        for(int i = 1; std::getline(in, line); i++) {
            lines.push_back(line);
        }

        if(in.bad()) {
            throw std::runtime_error("Runtime error in read_fen_from_file(const std::string&, const uint32): Badbit file.");
        }

        in.close();

    } catch(const std::exception& e) {
        throw;
    }

    return lines;
}

class ForwardKinTester : public ::testing::Test {
protected:


protected:
    virtual void SetUp() {

    }
};

TEST_F(ForwardKinTester, Callable) {
    Eigen::Matrix<double, 3, 1> phi {M_PI/4., M_PI/4., M_PI/4.};
    Eigen::Matrix<double, 6, 1> initial_guess{3.*M_PI/4., 0., 3.*M_PI/4., 0., 3.*M_PI/4., 0.};
    RobotParameters robot_params(0.5, 0.35, 0.125, 0.03);
    ConvergenceParameters convergence_params(1e-12, 1e-12, 1e-12, 1e-12, 100);
    ForwardKinematicsReturnType state = forward_kinematics(phi, initial_guess, robot_params, convergence_params);

    std::cout << state.robot_state << std::endl;
    std::cout << state.x_abserr << std::endl;
    std::cout << state.number_of_iterations << std::endl;
}

TEST_F(ForwardKinTester, FalsifyAgainstSimulation) {
    auto current_path = std::filesystem::current_path();
    std::string forward_kin_test_file = current_path.string() + "/../../test/forward_kin_artifacts/sim_results.txt";
    std::vector<std::string> lines;

    try {
        lines = read_file(forward_kin_test_file);
    } catch (...) {
        std::cout << "Exiting..." << std::endl;
    }

    std::vector<std::string> split_line;
    std::cout << "n = " << lines.size() << std::endl;

    StopWatch watch;

    watch.start();
    for (auto& line : lines) {
        split_line = split(line, ',');
        Eigen::Matrix<double, 9, 1> config_i(9, 1);
        for (unsigned i = 0; i < split_line.size(); ++i) {
            config_i(i, 0) = std::stod(split_line[i]);
        }

//        std::cout << config_i << std::endl;

        Eigen::Matrix<double, 3, 1> phi;
        phi(0,0) = config_i[0];
        phi(1,0) = config_i[3];
        phi(2,0) = config_i[6];

        Eigen::Matrix<double, 6, 1> initial_guess;
        initial_guess(0,0) = config_i[1];
        initial_guess(1,0) = config_i[2];
        initial_guess(2,0) = config_i[4];
        initial_guess(3,0) = config_i[5];
        initial_guess(4,0) = config_i[7];
        initial_guess(5,0) = config_i[8];

        RobotParameters robot_params(0.5, 0.35, 0.125, 0.03);
        ConvergenceParameters convergence_params(1e-12, 1e-12, 1e-12, 1e-12, 100);
        ForwardKinematicsReturnType state = forward_kinematics(phi, initial_guess, robot_params, convergence_params);

//        std::cout << state.robot_state << std::endl;
//        std::cout << state.x_abserr << std::endl;
//        std::cout << state.number_of_iterations << std::endl;

        std::cout << (state.robot_state - config_i).array().abs().mean() << std::endl << std::endl;
    }

    watch.stop();

    // Experiment conducted on March 25 on MacBook Pro (n = 5013)
    // - completeOrthogonalDecomposition: 2496.97 [ms]
    // - colPivHouseholderQr: 2270.49 [ms]
    // - fullPivLu: 1064.11 [ms] // -> use this
    // - partialPivLu: 1100.67 [ms]
    std::cout << (double)watch.duration() / 1e6 << " [ms]" << std::endl;
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
