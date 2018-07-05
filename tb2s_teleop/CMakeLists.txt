cmake_minimum_required(VERSION 2.8.3)
project(tb2s_teleop)

find_package(catkin REQUIRED COMPONENTS roscpp rospy geometry_msgs joy)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs joy
)

add_executable(turtlebot_teleop_joy src/turtlebot_joy.cpp)
target_link_libraries(turtlebot_teleop_joy ${catkin_LIBRARIES})
add_dependencies(turtlebot_teleop_joy ${catkin_EXPORTED_TARGETS})

install(DIRECTORY launch param
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS turtlebot_teleop_joy
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/turtlebot_teleop_key DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
