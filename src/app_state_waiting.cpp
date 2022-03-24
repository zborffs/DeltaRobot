#include "app_state_waiting.hpp"

AppStateWaiting::AppStateWaiting() {
    spdlog::get("delta_logger")->info("Creating AppStateWaiting...");
}

AppStateWaiting::~AppStateWaiting() {
    spdlog::get("delta_logger")->info("Destroying AppStateWaiting...");
}

bool AppStateWaiting::handle() {
    context_->transition_to(new AppStateThinking);
    return false;
}