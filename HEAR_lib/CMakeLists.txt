cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME HEAR_lib)

project(${PROJECT_NAME})

find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf2
)

include_directories(
    include    
    ${catkin_INCLUDE_DIRS}
)

file(GLOB HEAR_SRCs src/*)

add_library(hear_lib ${HEAR_SRCs})
add_dependencies(hear_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})