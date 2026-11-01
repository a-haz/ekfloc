cmake_minimum_required(VERSION 3.18)
project(ekfloc VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_library(ekfloc
    src/motion_model.cpp
    src/observation_model.cpp
    src/ekf.cpp
    src/metrics.cpp
    src/scenario.cpp
    src/simulator.cpp
)
target_include_directories(ekfloc PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(ekfloc PUBLIC Eigen3::Eigen)
target_compile_options(ekfloc PRIVATE -Wall -Wextra -Wpedantic -Werror)

function(ekfloc_apply_warnings target)
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Werror)
endfunction()

add_executable(ekf_loc apps/ekf_loc.cpp)
target_link_libraries(ekf_loc PRIVATE ekfloc)
target_include_directories(ekf_loc PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/apps)
ekfloc_apply_warnings(ekf_loc)

add_executable(ekf_loc_benchmark apps/benchmark.cpp)
target_link_libraries(ekf_loc_benchmark PRIVATE ekfloc)
target_include_directories(ekf_loc_benchmark PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/apps)
ekfloc_apply_warnings(ekf_loc_benchmark)

option(EKFLOC_BUILD_TESTS "Build GoogleTest unit tests" ON)
if(EKFLOC_BUILD_TESTS)
    include(FetchContent)
    FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.tar.gz
        URL_HASH SHA256=7b42b4d6ed48810c5362c265a17faebe90dc2373c885e5216439d37927f02926
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    enable_testing()
    add_executable(ekfloc_tests
        tests/test_angle.cpp
        tests/test_motion.cpp
        tests/test_observation.cpp
        tests/test_jacobians.cpp
        tests/test_ekf.cpp
        tests/test_consistency.cpp
        tests/test_simulator.cpp
        tests/test_metrics.cpp
    )
    target_link_libraries(ekfloc_tests PRIVATE ekfloc GTest::gtest_main)
    target_include_directories(ekfloc_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tests)
    ekfloc_apply_warnings(ekfloc_tests)
    include(GoogleTest)
    gtest_discover_tests(ekfloc_tests DISCOVERY_MODE PRE_TEST)
endif()
