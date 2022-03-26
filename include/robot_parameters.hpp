#ifndef DELTA_ROBOT_PARAMETERS_HPP
#define DELTA_ROBOT_PARAMETERS_HPP

#include <Eigen/Core>

class RobotParameters {
private:
    double distal_link_length_; // in meters
    double proximal_link_length_; // in meters
    double base_radius_; // in meters
    double platform_radius_; // in meters

public:
    RobotParameters() = default; // must have default constructor, so watch out
    RobotParameters(double distal_link_length, double proximal_link_length, double base_radius, double platform_radius);

    [[nodiscard]] inline double distal_link_length() const noexcept {
        return distal_link_length_;
    }

    [[nodiscard]] inline double proximal_link_length() const noexcept {
        return proximal_link_length_;
    }

    [[nodiscard]] inline double base_radius() const noexcept {
        return base_radius_;
    }

    [[nodiscard]] inline double platform_radius() const noexcept {
        return platform_radius_;
    }
};


#endif //DELTA_ROBOT_PARAMETERS_HPP
