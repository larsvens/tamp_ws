cmake_minimum_required(VERSION 2.8.3)
project(saasqp)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    common
    )

## Add catkin simple package
find_package(catkin_simple REQUIRED)
catkin_simple()

# activate c++ 11
IF(CMAKE_COMPILER_IS_GNUCC)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
ELSE()
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ENDIF()

# flags for speed (should already be enabled by default)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -fopenmp -O3")

# add lib for generated code using catkin simple
cs_add_library(rtisqp_solver
    rtisqp_export/qpoases/SRC/Bounds.cpp
    rtisqp_export/qpoases/SRC/Constraints.cpp
    rtisqp_export/qpoases/SRC/CyclingManager.cpp
    rtisqp_export/qpoases/SRC/Indexlist.cpp
    rtisqp_export/qpoases/SRC/MessageHandling.cpp
    rtisqp_export/qpoases/SRC/QProblem.cpp
    rtisqp_export/qpoases/SRC/QProblemB.cpp
    rtisqp_export/qpoases/SRC/SubjectTo.cpp
    rtisqp_export/qpoases/SRC/Utils.cpp
    rtisqp_export/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
    rtisqp_export/acado_qpoases_interface.cpp
    rtisqp_export/acado_integrator.c
    rtisqp_export/acado_solver.c
    rtisqp_export/acado_auxiliary_functions.c)

target_include_directories(rtisqp_solver PUBLIC
    rtisqp_export/
    rtisqp_export/qpoases
    rtisqp_export/qpoases/INCLUDE
    rtisqp_export/qpoases/SRC)

# add lib for rtisqp_wrapper.cpp
cs_add_library(rtisqp_wrapper
    src/rtisqp_wrapper.cpp)
target_link_libraries(rtisqp_wrapper
    rtisqp_solver)

# make an executable
cs_install()
cs_export()

# add executable for node
cs_add_executable(saasqp_node
    src/saasqp_node.cpp)
target_link_libraries(saasqp_node
    rtisqp_wrapper
    rtisqp_solver)



