cmake_minimum_required(VERSION 3.13)
project(lane_detection VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Collect sources (auto-reconfigure when files added/renamed)
file(GLOB SOURCES CONFIGURE_DEPENDS
    "VisualStudio/LaneKeeping/*.cpp"
    "VisualStudio/LaneKeeping/src/*.cpp"
)

add_executable(lane_detection ${SOURCES})

# Public include dir (the project keeps headers in VisualStudio/LaneKeeping/include)
target_include_directories(lane_detection PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/VisualStudio/LaneKeeping/include
)

# Find OpenCV and link
find_package(OpenCV REQUIRED)
if(OpenCV_FOUND)
    target_include_directories(lane_detection PRIVATE ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(lane_detection PRIVATE ${OpenCV_LIBS})
endif()

# Helpful compile flags
target_compile_options(lane_detection PRIVATE -Wall -Wextra)

# Add a configurable "run" target so you can pass runtime arguments from CMake.
# Usage:
#  cmake -S . -B build -DRUN_ARGS="/full/path/to/image.jpg"
#  cmake --build build --target run
set(RUN_ARGS "" CACHE STRING "Arguments to pass to lane_detection at run-time (e.g. '/path/to/image.jpg')")
add_custom_target(run
    COMMAND ${CMAKE_COMMAND} -E echo "Running lane_detection ${RUN_ARGS}"
    COMMAND ${CMAKE_COMMAND} -E chdir ${CMAKE_CURRENT_BINARY_DIR} $<TARGET_FILE:lane_detection> ${RUN_ARGS}
    DEPENDS lane_detection
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
)
