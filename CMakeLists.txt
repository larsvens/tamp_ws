cmake_minimum_required(VERSION 2.8.3)
project(saasqp)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

## Add catkin simple package
find_package(catkin_simple REQUIRED)
catkin_simple()

# add lib for generated code using catkin simple
cs_add_library(rtisqp_solver
  larsvens_experiment_export/qpoases/SRC/Bounds.cpp
  larsvens_experiment_export/qpoases/SRC/Constraints.cpp
  larsvens_experiment_export/qpoases/SRC/CyclingManager.cpp
  larsvens_experiment_export/qpoases/SRC/Indexlist.cpp
  larsvens_experiment_export/qpoases/SRC/MessageHandling.cpp
  larsvens_experiment_export/qpoases/SRC/QProblem.cpp
  larsvens_experiment_export/qpoases/SRC/QProblemB.cpp
  larsvens_experiment_export/qpoases/SRC/SubjectTo.cpp
  larsvens_experiment_export/qpoases/SRC/Utils.cpp
  larsvens_experiment_export/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
  larsvens_experiment_export/acado_qpoases_interface.cpp
  larsvens_experiment_export/acado_integrator.c
  larsvens_experiment_export/acado_solver.c
  larsvens_experiment_export/acado_auxiliary_functions.c)

target_include_directories(rtisqp_solver PUBLIC
  larsvens_experiment_export/
  larsvens_experiment_export/qpoases
  larsvens_experiment_export/qpoases/INCLUDE
  larsvens_experiment_export/qpoases/SRC)

target_link_libraries(rtisqp_solver)

# make an executable
cs_install()
cs_export()

cs_add_executable(saasqp_node
  src/saasqp_node.cpp)
target_link_libraries(saasqp_node
  rtisqp_solver)



