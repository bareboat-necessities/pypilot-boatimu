cmake_minimum_required(VERSION 3.16)
project(pypilot_boatimu LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(PYPILOT_DATA_MODEL_DIR "" CACHE PATH "Path to pypilot-data-model/src")
if(NOT PYPILOT_DATA_MODEL_DIR AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/deps/pypilot-data-model/src")
    set(PYPILOT_DATA_MODEL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/deps/pypilot-data-model/src")
endif()
if(NOT PYPILOT_DATA_MODEL_DIR AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../pypilot-data-model/src")
    set(PYPILOT_DATA_MODEL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../pypilot-data-model/src")
endif()

add_library(pypilot_boatimu INTERFACE)
target_include_directories(pypilot_boatimu INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
if(PYPILOT_DATA_MODEL_DIR)
    target_include_directories(pypilot_boatimu INTERFACE ${PYPILOT_DATA_MODEL_DIR})
endif()

add_executable(test_boatimu_samples tests/test_samples.cpp)
target_link_libraries(test_boatimu_samples PRIVATE pypilot_boatimu)

add_executable(test_boatimu_source_alignment tests/test_source_alignment.cpp)
target_link_libraries(test_boatimu_source_alignment PRIVATE pypilot_boatimu)

add_executable(test_boatimu_runtime tests/test_runtime.cpp)
target_link_libraries(test_boatimu_runtime PRIVATE pypilot_boatimu)

add_executable(test_boatimu_adapters tests/test_adapters.cpp)
target_link_libraries(test_boatimu_adapters PRIVATE pypilot_boatimu)

if(PYPILOT_DATA_MODEL_DIR)
    add_executable(test_boatimu_data_model_writer tests/test_data_model_writer.cpp)
    target_link_libraries(test_boatimu_data_model_writer PRIVATE pypilot_boatimu)
else()
    message(STATUS "Skipping BoatIMU data-model writer test; PYPILOT_DATA_MODEL_DIR is not set")
endif()

add_executable(boatimu_print_samples examples/linux/print_boatimu_samples.cpp)
target_link_libraries(boatimu_print_samples PRIVATE pypilot_boatimu)

enable_testing()
add_test(NAME test_boatimu_samples COMMAND test_boatimu_samples)
add_test(NAME test_boatimu_source_alignment COMMAND test_boatimu_source_alignment)
add_test(NAME test_boatimu_runtime COMMAND test_boatimu_runtime)
add_test(NAME test_boatimu_adapters COMMAND test_boatimu_adapters)
if(PYPILOT_DATA_MODEL_DIR)
    add_test(NAME test_boatimu_data_model_writer COMMAND test_boatimu_data_model_writer)
endif()
