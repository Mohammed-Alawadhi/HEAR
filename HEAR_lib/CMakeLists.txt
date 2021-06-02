cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME hear_flight_controller)

project(${PROJECT_NAME})

find_package(catkin REQUIRED COMPONENTS
    roscpp
    hear_hardware_interface
    hear_msgs
    geometry_msgs
    tf2
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    hear_msgs
    roscpp
    geometry_msgs
    hear_hardware_interface
    hear_msgs
    tf2
)

include_directories(
    include    
    ${catkin_INCLUDE_DIRS}
)

file(GLOB HEAR_SRCs src/*)

add_executable(hear_fc_node ${HEAR_SRCs})
add_dependencies(hear_fc_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(hear_fc_node ${catkin_LIBRARIES})