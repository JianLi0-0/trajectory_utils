cmake_minimum_required(VERSION 3.0.2)
project(trajectory_utils)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
        roscpp
)

find_package(ruckig REQUIRED)

find_package(Python3 REQUIRED COMPONENTS
        Interpreter
        Development
        NumPy
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES trajectory_utils
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
        include
        src
        ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
        src/trajectory_info.cpp
        src/discretized_path.cpp
        src/discretized_trajectory.cpp
        src/reference_line.cpp
        src/math/discrete_points_math.cpp
        src/math/linear_interpolation.cpp
        src/math/math_utils.cpp
        src/math/vec2d.cpp
        src/math/line_segment2d.cpp
        src/proto/pnc_point.pb.cc)

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ruckig::ruckig
        protobuf
)

add_executable(trajectory_info_test src/test.cpp)
target_link_libraries(trajectory_info_test
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ruckig::ruckig
        protobuf
        Python3::Python
        Python3::Module
        Python3::NumPy
)