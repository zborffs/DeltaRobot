#ifndef DELTA_APP_STATE_WAITING_HPP
#define DELTA_APP_STATE_WAITING_HPP

/// Logging includes
#include <spdlog/spdlog.h>

/// internal includes
#include "app.hpp"
#include "app_state_thinking.hpp"
#include "app_state_shutting_down.hpp"
#include "robot_state.hpp"

class AppStateWaiting : public AppState {
private:

public:
    explicit AppStateWaiting();
    ~AppStateWaiting() override;
    bool handle() override;
};


#endif //DELTA_APP_STATE_WAITING_HPP
