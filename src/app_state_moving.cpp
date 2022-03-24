#include "app_state_moving.hpp"

AppStateMoving::AppStateMoving() {
    spdlog::get("delta_logger")->info("Creating AppStateMoving...");
}

AppStateMoving::~AppStateMoving() {
    spdlog::get("delta_logger")->info("Destroying AppStateMoving...");
}

bool AppStateMoving::handle() {
    // what do we want to do while the robot is in the "moving" app state?
    // -> make calls to the RobotState object to figure out what the motor angles are, what the motor velocities are,
    // etc.
    // -> make calls to EnvironmentState object to figure out where pieces are in cartesian base-frame, what the
    // chess position is, etc.
    // -> when do we quit? when do we transition to the next state (i.e. the waiting state?)
    context_->transition_to(new AppStateShuttingDown);
    return false;
}