cmake_minimum_required(VERSION 3.13)
project(lane_detection VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

add_executable(lane_detection
    src/main.cpp
    src/LaneDetection.cpp
    src/lane_pipe_writer.cpp
)

target_include_directories(lane_detection PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(lane_detection PRIVATE
    ${OpenCV_LIBS}
)

target_compile_options(lane_detection PRIVATE -Wall -Wextra)

# Add a configurable "run" target so you can pass runtime arguments from CMake.
set(RUN_ARGS "" CACHE STRING "Arguments to pass to lane_detection at run-time")

add_custom_target(run
    COMMAND ${CMAKE_COMMAND} -E echo "Running lane_detection ${RUN_ARGS}"
    COMMAND ${CMAKE_COMMAND} -E chdir ${CMAKE_CURRENT_BINARY_DIR} $<TARGET_FILE:lane_detection> ${RUN_ARGS}
    DEPENDS lane_detection
)
