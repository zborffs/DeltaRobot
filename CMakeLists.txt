cmake_minimum_required(VERSION 3.13)
project(delta)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Must include to avoid failing on Linux with std::thread
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# Import Packages
find_package(GTest REQUIRED) # Unit-Testing Library
# find_package(benchmark REQUIRED) # Benchmarking Library
find_package(spdlog REQUIRED) # Logging Library
find_package(OpenCV REQUIRED)

# Include Library Directories
include_directories(${GTEST_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
set(EIGEN_DIR /usr/local/include/eigen3)
include_directories(${EIGEN_DIR})
include_directories(include)

# Options
option(BUILD_TESTS "Build Tests" ON)

if (JETSON_BUILD)
    add_compile_definitions(JETSON_BUILD)
    #    add_definitions( -DJETSON_BUILD )
    #    get_directory_property( DirDefs DIRECTORY ${CMAKE_SOURCE_DIR} COMPILE_DEFINITIONS )
    message("-- Compiling for Jetson")
endif()

if (ASan)
    message("-- Compiling with Address Sanitizer")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fsanitize=address")
endif()

if (ThSan)
    message("-- Compiling with Thread Sanitizer")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fsanitize=thread")
endif()

if (USan)
    message("-- Compiling with Undefined Sanitizer")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fsanitize=undefined")
endif()

if (CMAKE_CXX_COMPILER_ID MATCHES "Clang" OR CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wshadow -pedantic") # -Werror
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g") # -O3
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4 /WX")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /O3")
endif(CMAKE_CXX_COMPILER_ID MATCHES "Clang" OR CMAKE_CXX_COMPILER_ID STREQUAL "GNU")

# Put all source files excluding main.cpp into library source variable
set(DELTA_LIB_SRCS
        src/forward_kinematics.cpp
        src/position_recognizer.cpp
        src/robot_state.cpp
        src/robot_parameters.cpp
        src/environment_state.cpp
        src/force_sensor.cpp
        src/app.cpp
        src/app_state_init.cpp
        src/app_state_waiting.cpp
        src/app_state_thinking.cpp
        src/app_state_moving.cpp
        src/app_state_shutting_down.cpp
#        src/main.cpp
)

# Create library, link external libraries to it
add_library(delta-lib ${DELTA_LIB_SRCS})
set_target_properties(delta-lib PROPERTIES PREFIX "")
target_link_libraries(delta-lib Threads::Threads spdlog::spdlog ${OpenCV_LIBS})
target_include_directories(delta-lib PUBLIC ${CMAKE_SOURCE_DIR}/include ${CMAKE_BINARY_DIR}/include ${SPDLOG_INCLUDE_DIR})

# Create main executable
add_executable(delta src/main.cpp)
target_link_libraries(delta delta-lib Threads::Threads spdlog::spdlog ${OpenCV_LIBS})
install(TARGETS delta DESTINATION bin)

if (BUILD_TESTS)
    message("-- Building tests")
    add_subdirectory(test)
endif()