#ifndef DELTA_APP_HPP
#define DELTA_APP_HPP

/// Logging Includes
#include <spdlog/spdlog.h>

/// Internal Includes
#include "robot_parameters.hpp"
#include "robot_state.hpp"
#include "environment_state.hpp"


class App; // forward declare "App" class, so it may be referenced in the "AppState" class

/**
 * "AppState" class is an abstract parent class of each of the AppState classes (necessary for **State Design Pattern**).
 */
class AppState {
protected:
    App* context_{};
public:
    virtual ~AppState() = default;
    virtual bool handle() = 0;
    void set_context(App* context);
};

/**
 * "App" class is the context main context that the application runs it. It manages the objects responsible for holding
 * robot and environment state information and maintains which "AppState" we are in (for State Design Pattern).
 */
class App {
protected:
    friend class AppStateInit;
    friend class AppStateWaiting;
    friend class AppStateThinking;
    friend class AppStateMoving;

private:
    // State Design-Pattern (current state)
    std::unique_ptr<AppState> app_state_;

    // member variables
    RobotParameters robot_params_; // the robot's parameters (lengths and masses)
    RobotState robot_state_; // the "state" of the robot in the Lagrangian sense (configuration + velocity variables)
    EnvironmentState env_state_; // the "state" of the environment (where pieces are, what the position is, etc.)

public:
    explicit App(std::unique_ptr<AppState> init_state);
    void transition_to(AppState* state);
    void operator()();
};

#endif //DELTA_APP_HPP
