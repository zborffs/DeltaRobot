#ifndef DELTA_APP_STATE_MOVING_HPP
#define DELTA_APP_STATE_MOVING_HPP

/// Logging includes
#include <spdlog/spdlog.h>

/// internal includes
#include "app.hpp"
#include "app_state_waiting.hpp"
#include "app_state_shutting_down.hpp"
#include "robot_state.hpp"

class AppStateMoving : public AppState {
private:

public:
    explicit AppStateMoving();
    ~AppStateMoving() override;
    bool handle() override;
};

#endif //DELTA_APP_STATE_MOVING_HPP
