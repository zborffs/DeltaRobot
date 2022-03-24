#ifndef DELTA_APP_STATE_SHUTTING_DOWN_HPP
#define DELTA_APP_STATE_SHUTTING_DOWN_HPP

/// Logging includes
#include <spdlog/spdlog.h>

/// internal includes
#include "app.hpp"
#include "app_state_moving.hpp"
#include "robot_state.hpp"

class AppStateShuttingDown : public AppState {
private:

public:
    explicit AppStateShuttingDown();
    ~AppStateShuttingDown() override;
    bool handle() override;
};

#endif //DELTA_APP_STATE_SHUTTING_DOWN_HPP
