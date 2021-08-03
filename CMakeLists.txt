cmake_minimum_required(VERSION 3.5.1)
project(redundancy_solver)

add_compile_options(-std=c++11)

#find_package(catkin_simple REQUIRED)
#catkin_simple(ALL_DEPS_REQUIRED)

set(CMAKE_BUILD_TYPE Release)

if("${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "x86_64" AND NOT APPLE)
  add_definitions(-fPIC)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  trajectory_msgs
  moveit_core
  moveit_ros_planning_interface
  orocos_kdl
  message_generation
)

## Generate services in the 'srv' folder
add_service_files(DIRECTORY srv
 FILES
 GetGoals.srv
 SimulateGoals.srv
 ChangeSafetyDistance.srv
 GetGoalsJoints.srv
 SimulateGoalsJoints.srv
)
## Generate added messages and services with any dependencies listed here
generate_messages(
 DEPENDENCIES
 std_msgs
 geometry_msgs
 sensor_msgs
)

include_directories(
 	include
 	${catkin_INCLUDE_DIRS}
)

include_directories(lib)

#############
# LIBRARIES #
#############
add_library(redundancy_solver
  src/redundancy_solver.cpp
)

add_library(redundancy_solver_ur
  src/redundancy_solver_ur.cpp
)

add_library(solver_simulator
  src/solver_simulator.cpp
)


add_library(cvxgen_solver 
  lib/ldl.c
  lib/solver.c
  lib/matrix_support.c
  lib/util.c
  lib/variables_definition.c
)

target_link_libraries(redundancy_solver cvxgen_solver)

target_link_libraries(redundancy_solver_ur cvxgen_solver)

target_link_libraries(solver_simulator cvxgen_solver)

############
# BINARIES #
############
add_executable(redundancy_solver_node
  src/redundancy_solver_node.cpp
)

add_executable(solver_simulator_node
  src/solver_simulator_node.cpp
)

add_executable(test_goals
  src/test_goals.cpp 
)

set_target_properties(redundancy_solver_node PROPERTIES COMPILE_FLAGS "-O3")

add_executable(frame_tf_broadcaster src/frame_tf_broadcaster.cpp)
target_link_libraries(frame_tf_broadcaster ${catkin_LIBRARIES})

target_link_libraries(solver_simulator_node
	solver_simulator
	cvxgen_solver
	${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${GSTREAMER_LIBRARIES}
)

target_link_libraries(redundancy_solver_node
	redundancy_solver
  redundancy_solver_ur
	cvxgen_solver
	${PROJECT_NAME}
	${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${GSTREAMER_LIBRARIES}
)

target_link_libraries(test_goals
  ${catkin_LIBRARIES}
)

add_dependencies(redundancy_solver_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(solver_simulator_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(test_goals ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(frame_tf_broadcaster ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

##########
# EXPORT #
##########

catkin_package(
  CATKIN_DEPENDS message_runtime
 	INCLUDE_DIRS include
	lib
  LIBRARIES redundancy_solver
	cvxgen_solver
  solver_simulator
)

#cs_install()
#cs_export()
