cmake_minimum_required(VERSION 2.8.3)
project(tb2s_pantilt)

find_package(catkin REQUIRED COMPONENTS message_generation rospy)

# This is for python modules being loaded by other programs
# make sure to have setup.py in project root, and __init__.py in the scripts dir
catkin_python_setup()

# generate_messages()
catkin_package(
  CATKIN_DEPENDS dynamixel_controllers 
)

install(DIRECTORY launch config udev
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# This is only for stand-alone executable scripts (not being loaded by another python program!)
catkin_install_python(PROGRAMS scripts/dynamixel_joint_state_publisher.py  scripts/dynamixel_relax_all_servos.py  scripts/joy_to_dynamixel.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

