#ifndef DELTA_APP_STATE_INIT_HPP
#define DELTA_APP_STATE_INIT_HPP

/// Logging includes
#include <spdlog/spdlog.h>

/// internal includes
#include "app.hpp"
#include "app_state_waiting.hpp"
#include "app_state_shutting_down.hpp"
#include "robot_state.hpp"

class AppStateInit : public AppState {
private:

public:
    explicit AppStateInit();
    ~AppStateInit() override;
    bool handle() override;
};

#endif //DELTA_APP_STATE_INIT_HPP
