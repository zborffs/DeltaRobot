#ifndef DELTA_APP_STATE_THINKING_HPP
#define DELTA_APP_STATE_THINKING_HPP

/// Logging includes
#include <spdlog/spdlog.h>

/// internal includes
#include "app.hpp"
#include "app_state_moving.hpp"
#include "app_state_shutting_down.hpp"
#include "robot_state.hpp"

class AppStateThinking : public AppState {
private:

public:
    explicit AppStateThinking();
    ~AppStateThinking() override;
    bool handle() override;
};

#endif //DELTA_APP_STATE_THINKING_HPP
