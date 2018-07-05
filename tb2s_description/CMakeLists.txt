cmake_minimum_required(VERSION 2.8.3)

project(tb2s_description)
find_package(catkin REQUIRED COMPONENTS urdf xacro)

catkin_package(
   CATKIN_DEPENDS urdf xacro
)

install(DIRECTORY launch urdf rviz meshes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
