#include "app_state_thinking.hpp"

AppStateThinking::AppStateThinking() {
    spdlog::get("delta_logger")->info("Creating AppStateThinking...");
}

AppStateThinking::~AppStateThinking() {
    spdlog::get("delta_logger")->info("Destroying AppStateThinking...");
}

bool AppStateThinking::handle() {

    // what do we want to do while the robot is in the "thinking" app state?
    // -> dispatch a go request to another process running the Chess Engine
    // -> make sure we reject disturbances given the state of the robot as a dynamical system given the sensor
    // measurements
    // -> when do we quit? when do we transition to the next state (i.e. the moving state?)
    context_->transition_to(new AppStateMoving);
    return false;
}