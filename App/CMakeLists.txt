cmake_minimum_required(VERSION 2.8.11)
project(EuRoC_test)
# =========================
# Example
# =========================
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/../bin)

add_executable(ice_ba
    ../App/EuRoC_test.cpp
)

target_link_libraries(ice_ba
    IBA
    OF
)
