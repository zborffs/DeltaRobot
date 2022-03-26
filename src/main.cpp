/// Standard Library Includes
#include <iostream>

/// Logging Includes
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/basic_file_sink.h>

/// Internal Includes
#include "app.hpp"
#include "app_state_init.hpp"

#define LOGGER_INITIALIZATION_FAILED -1
#define SUCCESS 0

/**
 * sets up the console and log-file sinks for the logger used throughout the codebase
 * @param  logfile_path the path to the logfile
 * @return true means the initialization was successful, false otherwise
 */
bool init_logger(const std::string& logfile_path) {
    try {
        // Setup the console sink
        auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        console_sink->set_level(spdlog::level::warn);
        console_sink->set_pattern("[%D %H:%M:%S] [%^%l%$] [Thread %t] [File:Line %@] [Function: %!] %v");
        console_sink->set_color_mode(spdlog::color_mode::always);

        // setup the file sink
        auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(logfile_path, true);
        file_sink->set_level(spdlog::level::trace);

        // setup the logger using both sinks
        spdlog::logger logger("delta_logger", {console_sink, file_sink});
        logger.set_level(spdlog::level::debug);
        spdlog::set_default_logger(std::make_shared<spdlog::logger>("delta_logger", spdlog::sinks_init_list({console_sink, file_sink})));
    } catch (const spdlog::spdlog_ex& ex) {
        std::cout << "Logger initialization failed: " << ex.what() << std::endl;
        return false; // if we fail to initialize the logger, return false
    }

    return true;
}

#include "forward_kinematics.hpp"

int main() {
    // initialize the logger and path to the log files
    std::string logfile_path("../logs/delta.log");
    if (!init_logger(logfile_path)) {
        return LOGGER_INITIALIZATION_FAILED; // if the initialization of the logger fails, just quit immediately
    }

//    Eigen::Matrix<double, 9, 1> d = forward_kin()

    // start the main application loop,
//    App app(std::make_unique<AppStateInit>());
//    app();

    return SUCCESS;
}
