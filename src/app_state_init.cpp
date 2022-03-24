#include "app_state_init.hpp"

AppStateInit::AppStateInit() {
    spdlog::get("delta_logger")->info("Creating AppStateInit...");
}

AppStateInit::~AppStateInit() {
    spdlog::get("delta_logger")->info("Destroying AppStateInit...");
}

bool AppStateInit::handle() {
    context_->transition_to(new AppStateWaiting);
    return false;
}