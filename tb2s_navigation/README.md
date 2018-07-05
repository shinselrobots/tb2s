# tb2s_navigaton

1 minimal.launch:  basic, flat costmap running mostly defaults

2 navigation.launch: multi-layer costmap with dynamic obstacles.
  - most of this stuff copied from turtlebot_navigation, then modified for tb2s.
  - in particular, in costmap_common_params.yaml: z_voxels and max_obstacle_height must be modified for the height of the robot

3 How to launch for testing:
  - roslaunch tb2s robot.launch
  - roslaunch tb2s_navigation navigation.launch (assumes robot.launch does not already do this)
  - roslaunch tb2s_navigation view_map.launch


4. Variations:
  - navigation.launch: normal navigation, full stack wit
  - minimal.launch:    basic navigation using mostly defaults
  - no_map.launch:     local navigation, useful for "follow-me" type behavior (WIP)
