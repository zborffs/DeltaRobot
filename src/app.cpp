#include "app.hpp"

/**
 * sets the "context" (in the "State Design Pattern" sense) as a pointer to the current app object, so the current
 * AppState can access app data
 * @param context a pointer to the App object
 */
void AppState::set_context(App* context) {
    this->context_ = context;
}

/**
 * sets internal current state to the state being passed in, and sets the context of the current state to the app
 * @param init_state
 */
App::App(std::unique_ptr<AppState> init_state) : app_state_(std::move(init_state)) {
    app_state_->set_context(this);
}

/**
 * transitions to a completely different AppState, destroying the old AppState (and its data)
 * @param state
 */
void App::transition_to(AppState* state) {
    app_state_.reset(state); // destroy the old AppState, and put the new state in its place
    app_state_->set_context(this); // set the context of the AppState to the App
}

/**
 * runs through the main Application loop
 */
void App::operator()() {
    bool quit{false};
    while (!quit) {
        // multirate?
        robot_state_.update();
        env_state_.update();
        quit = app_state_->handle();
    }
}