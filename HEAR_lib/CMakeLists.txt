cmake_minimum_required(VERSION 2.8.3)

# set(PROJECT_NAME hear_flight_controller)

project(hear_flight_controller)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    hear_hardware_interface
    hear_msgs
    geometry_msgs
    tf2_geometry_msgs
    tf2
    tf2_ros
    nodelet
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
    tf2_ros
    tf2_geometry_msgs
    nodelet
)

include_directories(
    include
    HEAR_nodelet    
    ${catkin_INCLUDE_DIRS}
)

file(GLOB HEAR_SRCs src/*.cpp)

add_library(${PROJECT_NAME} ${HEAR_SRCs})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

add_executable(opti_node HEAR_nodelet/opti_node.cpp)
add_dependencies(opti_node ${catkin_LIBRARIES})
target_link_libraries(opti_node ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(slam_node HEAR_nodelet/slam_node.cpp)
add_dependencies(slam_node ${catkin_LIBRARIES})
target_link_libraries(slam_node ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(outer_sys_node HEAR_nodelet/outer_sys_node.cpp)
add_dependencies(outer_sys_node ${catkin_LIBRARIES})
target_link_libraries(outer_sys_node ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(inner_sys_node HEAR_nodelet/inner_sys_node.cpp)
add_dependencies(inner_sys_node ${catkin_LIBRARIES})
target_link_libraries(inner_sys_node ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(actuation_sys_node HEAR_nodelet/actuation_sys_node.cpp)
add_dependencies(actuation_sys_node ${catkin_LIBRARIES})
target_link_libraries(actuation_sys_node ${PROJECT_NAME} ${catkin_LIBRARIES})


add_library(outer_sys_nodelet HEAR_nodelet/OuterSysNodelet.cpp)
add_dependencies(outer_sys_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(outer_sys_nodelet
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

add_library(inner_sys_nodelet HEAR_nodelet/InnerSysNodelet.cpp)
add_dependencies(inner_sys_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(inner_sys_nodelet
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

add_library(actuation_sys_nodelet HEAR_nodelet/ActuationSysNodelet.cpp)
add_dependencies(actuation_sys_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(actuation_sys_nodelet
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

add_library(opti_nodelet HEAR_nodelet/OptiNodelet.cpp)
add_dependencies(opti_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(opti_nodelet
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

install(TARGETS outer_sys_nodelet inner_sys_nodelet actuation_sys_nodelet opti_nodelet ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(FILES outer_sys_nodelet.xml inner_sys_nodelet.xml actuation_sys_nodelet.xml opti_nodelet.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)