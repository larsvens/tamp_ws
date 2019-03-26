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

target_link_libraries(rtisqp_solver)

# make an executable
cs_install()
cs_export()

cs_add_executable(saasqp_node
  src/saasqp_node.cpp)
target_link_libraries(saasqp_node
  rtisqp_solver)



