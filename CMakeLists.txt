set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

cmake_minimum_required(VERSION 2.8.3)
project(s8_turner)

find_package(catkin REQUIRED COMPONENTS roscpp s8_msgs geometry_msgs s8_common_node s8_utils actionlib actionlib_msgs s8_motor_controller)
find_package(Boost REQUIRED COMPONENTS system)

add_action_files(
  DIRECTORY action
  FILES Turn.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs
)

catkin_package(
  DEPENDS
  CATKIN_DEPENDS roscpp s8_msgs geometry_msgs s8_common_node s8_utils actionlib_msgs actionlib s8_motor_controller
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(s8_turner_node src/turner.cpp)
target_link_libraries(s8_turner_node ${catkin_LIBRARIES})
add_dependencies(s8_turner_node ${s8_turner_EXPORTED_TARGETS})
