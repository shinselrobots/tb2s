#!/bin/bash
#
# copy scripts to the correct location
## THIS IS A KLUDGE
#for some reason, I can't get other catkin projects to find my scripts!
#So, for now, putting the scripts here, and manually copying to:
#/home/system/catkin_robot/devel/lib/python2.7/dist-packages

rm ~/catkin_robot/devel/lib/python2.7/dist-packages/sheldon_servos/*.pyc
cp *.py ~/catkin_robot/devel/lib/python2.7/dist-packages/tb2s_pantilt
echo "~/catkin_robot/devel/lib/python2.7/dist-packages/tb2s_pantilt:"
ls -la ~/catkin_robot/devel/lib/python2.7/dist-packages/tb2s_pantilt

