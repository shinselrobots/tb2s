# tb2s_description
# Model of the robot (URDF and meshes) and tests (launch and rviz) 

Test URDF model:
  For verifying URDF is correct
    - sets rviz to use "base_link"
    - launchs "joint_state_publisher" to simulate robot state that would normally come from the real robot

    roslaunch tb2_description test_model.launch


View live model of running robot:
  To view robot pose, feedback from servos, sensors, etc.
    - this is the state that the robot thinks is is in.

    roslaunch tb2s robot.launch
    roslaunch tbs2_description view_model.launch

