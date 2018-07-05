// Copyright 2018 Matt Curfman, Dave Shinsel
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <angles/angles.h>
#include <string>

#if 0
tf:Quaternion q(x,y,z,w);
double yaw,pitch,roll;
btMatrix3x3(q).getEulerYPR(yaw,pitch,roll);
#endif


void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  double yaw, pitch, roll;

  ROS_INFO("Pose Orientation: x: [%f], y: [%f], z: [%f], w: [%f]", 
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, 
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

  tf::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, 
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

  tf::Matrix3x3(q).getEulerYPR(yaw, pitch, roll);

  ROS_INFO("Heading: %f radians, %f degrees", yaw, angles::to_degrees(yaw));
}

int main( int argc, char *argv[] )
{
  ros::init( argc, argv, "quaternion_to_euler" );
  ros::NodeHandle nh;

  ros::Subscriber odometrySubscriber = nh.subscribe("/odom", 1, &odometryCallback);
  ros::spin();

  return 0;
}
