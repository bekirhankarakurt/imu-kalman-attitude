cmake_minimum_required(VERSION 3.16)
project(imu_kalman_attitude VERSION 1.0.0 LANGUAGES CXX)

if(NOT UNIX)
  message(FATAL_ERROR "imu-kalman-attitude requires a POSIX platform (Linux or macOS)")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(IMU_BUILD_TESTS "Build the unit tests" ON)
option(IMU_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)
option(IMU_SANITIZE "Enable AddressSanitizer and UBSan" OFF)

find_package(Threads REQUIRED)

add_library(imu_core STATIC
  src/filters.cpp
  src/simulator.cpp
  src/evaluation.cpp
  src/stream_format.cpp
  src/net.cpp
  src/sources.cpp
  src/state.cpp
  src/http_server.cpp
  src/viewer_html.cpp
  src/cli.cpp
  src/app.cpp)
target_include_directories(imu_core PUBLIC include)
target_link_libraries(imu_core PUBLIC Threads::Threads)

add_executable(imu_attitude src/main.cpp)
target_link_libraries(imu_attitude PRIVATE imu_core)

foreach(target imu_core imu_attitude)
  target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Wshadow -Wconversion)
  if(IMU_WARNINGS_AS_ERRORS)
    target_compile_options(${target} PRIVATE -Werror)
  endif()
  if(IMU_SANITIZE)
    target_compile_options(${target} PUBLIC -fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover=undefined)
    target_link_options(${target} PUBLIC -fsanitize=address,undefined)
  endif()
endforeach()

if(IMU_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
