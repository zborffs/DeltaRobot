#include "robot_parameters.hpp"

RobotParameters::RobotParameters(double distal_link_length, double proximal_link_length, double base_radius, double platform_radius)
: distal_link_length_(distal_link_length), proximal_link_length_(proximal_link_length), base_radius_(base_radius), platform_radius_(platform_radius) {

    // lengths cannot be negative or zero.
    assert(distal_link_length > 0);
    assert(proximal_link_length > 0);
    assert(base_radius > 0);
    assert(platform_radius > 0);

}
