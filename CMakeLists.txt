cmake_minimum_required(VERSION 3.0.2)
project(trajectory_utils)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找 qpOASES 库，但不要求必须找到
find_library(QPOASES_LIBRARY qpOASES)
# 如果找到库，设置预处理器定义
if(QPOASES_LIBRARY)
    message(STATUS "Found qpOASES library: ${QPOASES_LIBRARY}")
    add_definitions(-DHAVE_QPOASES)
else()
    message(STATUS "qpOASES library not found, will use OSQP instead")
endif()

find_package(catkin REQUIRED COMPONENTS
        roscpp
)

find_package(ruckig REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Python3 REQUIRED COMPONENTS
        Interpreter
        Development
        NumPy
)

find_package(Protobuf REQUIRED)
set(PROTO_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/proto/pnc_point.proto)
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})

add_subdirectory(qp_solver/osqp)

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
        ${EIGEN3_INCLUDE_DIR}
        qp_solver/osqp
        qp_solver/osqp/include
        qp_solver/osqp-eigen/include
)

set(OSQP_LIBRARY osqp)
add_subdirectory(qp_solver/osqp-eigen)

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
        src/mpc.cpp
        ${PROTO_SRCS})

target_include_directories(${PROJECT_NAME} PUBLIC
        src
        ${Protobuf_INCLUDE_DIRS}
        ${CMAKE_CURRENT_BINARY_DIR}
)

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ruckig::ruckig
        ${Protobuf_LIBRARIES}
        Python3::Python
        Python3::Module
        Python3::NumPy
        osqp
        OsqpEigen
        $<$<BOOL:${QPOASES_LIBRARY}>:${QPOASES_LIBRARY}>
)

#add_executable(trajectory_info_test src/test.cpp)
#target_link_libraries(trajectory_info_test
#        ${PROJECT_NAME}
#        ${catkin_LIBRARIES}
#        ruckig::ruckig
#        protobuf
#        Python3::Python
#        Python3::Module
#        Python3::NumPy
#)

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)