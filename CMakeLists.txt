cmake_minimum_required(VERSION 3.16)
project(pypilot_boatimu LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(pypilot_boatimu INTERFACE)
target_include_directories(pypilot_boatimu INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

add_executable(test_boatimu_samples tests/test_samples.cpp)
target_link_libraries(test_boatimu_samples PRIVATE pypilot_boatimu)

add_executable(boatimu_print_samples examples/linux/print_boatimu_samples.cpp)
target_link_libraries(boatimu_print_samples PRIVATE pypilot_boatimu)

enable_testing()
add_test(NAME test_boatimu_samples COMMAND test_boatimu_samples)
