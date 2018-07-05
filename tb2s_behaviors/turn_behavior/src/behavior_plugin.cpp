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
#include <pluginlib/class_list_macros.h>
#include <behavior_common/behavior.h>
#include <behavior_common/behavior_common.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <angles/angles.h>
#include <string>


namespace behavior_plugin 
{

  enum State
  {
  	idle,
  	initializing_start_position,
  	turning,
    ending
  };
	
  class TurnService : public behavior_common::BehaviorActionServiceBase
  {
  public:
    TurnService(std::string service_name) :
      behavior_common::BehaviorActionServiceBase(service_name),
      state_(idle),
      DEFAULT_SPEED(0.9),
      rotation_speed_(DEFAULT_SPEED),
      ramp_down_fudge_(0.0),
      turn_progress_(0.0),
      goal_turn_amount_(0.0),    
      last_angle_(0.0)   
    {
      // Init publishers and subscribers
      // rotation is monitored in odometryCallback
      cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/realsense", 1); 
    }

    virtual ~TurnService()
    {
      // Stop moving before exiting
      state_ = idle;    
      stop();
    }

    virtual void StartBehavior(const char *param1, const char *param2)
    {
      // TODO: Add the implementation of your behavior here.  You can either
      // register additional callbacks which will be invoked while your behavior
      // is running, or you can run the entire behavior here (watching for a
      // possible request to prempt your behavior below)
 
      // Param1: amount in degrees ()
      // Param2: speed
      ROS_INFO("TURN Param1 = %s, Param2= %s", param1, param2);

      double goal_turn_amount_degrees = atof(param1);
      goal_turn_amount_ = angles::from_degrees(goal_turn_amount_degrees);

      rotation_speed_ = fabs(atof(param2));
      // Kludge to compensate for velocity smoother ramp down
      ramp_down_fudge_ = angles::from_degrees(10.0) * rotation_speed_; 


      if(goal_turn_amount_ < 0.0)
      {
        rotation_speed_ *= -1.0;
      }

      ROS_INFO("TURN: Goal turn amount (rad, deg): %f, %f", goal_turn_amount_, goal_turn_amount_degrees);
      ROS_INFO("TURN: Turn speed: %f", rotation_speed_);
      odometrySubscriber_ = nh_.subscribe("/odom", 1, &TurnService::odometryCallback, this);
      state_ = initializing_start_position;
    }

    virtual void PremptBehavior()
    {
      // TODO: Add code which prempts your running behavior here.
      state_ = idle;
      stop();
      odometrySubscriber_.shutdown();
    }

    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
      // Called on each odometry update (~50hz)
      double yaw, pitch, roll;

      #if 0
      ROS_INFO("Pose Orientation: x: [%f], y: [%f], z: [%f], w: [%f]", 
	      msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, 
	      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
      #endif

      tf::Quaternion q(
	      msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, 
	      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

      tf::Matrix3x3(q).getEulerYPR(yaw, pitch, roll);

      if(initializing_start_position == state_)
      {
        turn_progress_ = 0.0;
        last_angle_ = angles::normalize_angle_positive(yaw); //  0-360
        ROS_INFO("TURN: Start Angle: %f radians, %f degrees", last_angle_, angles::to_degrees(last_angle_));
        ROS_INFO("    DBG: Heading: %f radians, %f degrees", yaw, angles::to_degrees(yaw));

        // Start moving
        geometry_msgs::Twist cmd;
        cmd.linear.x = 0;
        cmd.angular.z = rotation_speed_;  // TODO - set speed here (initially constant, then variable)
        cmd_vel_pub_.publish(cmd);

        state_ = turning;

      }
      else if(turning == state_)
      {
        // Monitor movement, stop when within goal postion
        double current_angle = angles::normalize_angle_positive(yaw); //  0-360
        double angle_delta = angles::shortest_angular_distance(last_angle_, current_angle);
        turn_progress_ += angle_delta;
        last_angle_ = current_angle;
        double turn_remaining = fabs(goal_turn_amount_) - (fabs(turn_progress_) + ramp_down_fudge_);

        if( fabs(turn_progress_) > 0.03) // display once turn actually starts
        {
  	  	  ROS_INFO("TURN Progress: %f rad, %f deg", 
            turn_progress_, angles::to_degrees(turn_progress_));

          ROS_INFO("TURN Remaining: %f rad, %f deg", 
            turn_remaining, angles::to_degrees(turn_remaining));

          ROS_INFO("    DBG: Heading: %f radians, %f degrees", yaw, angles::to_degrees(yaw));
        }

        if(turn_remaining < 0.0)
        {
		      ROS_INFO("TURN Complete!  Stopping.");
          stop();
    	    // Mark behavior complete
          BehaviorComplete();  
          state_ = ending;
        }
        else
        {
          // continue to turn
          geometry_msgs::Twist cmd;
          cmd.linear.x = 0;
          cmd.angular.z = rotation_speed_;  // TODO - set speed here (initially constant, then variable)
          cmd_vel_pub_.publish(cmd);

        }
      }
      else if (ending == state_)
      {
        ROS_INFO("    DBG: Final Heading: %f radians, %f degrees", yaw, angles::to_degrees(yaw));
        state_ = idle;
      }
    }



    // Utility Functions

    void stop()
    {
      geometry_msgs::Twist cmd;
      cmd.linear.x = cmd.angular.z = 0;  
      cmd_vel_pub_.publish(cmd); 
    }

    
/***
    double NormalizeAngleOld(double angle)
    {
      double normalized = angle;
      while(normalized > pi) 
      {
          normalized -= pi * 2.0;
      }
      while(normalized < -pi)
      {
        normalized += pi * 2.0; 
      }
      return normalized;
    }
***/


  protected:
    ros::Subscriber odometrySubscriber_;
    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;

    State  state_;
    const double DEFAULT_SPEED;
    double rotation_speed_;
    double ramp_down_fudge_;
    double turn_progress_;
    double goal_turn_amount_;
    double last_angle_;
 

  };

    CPP_BEHAVIOR_PLUGIN(TurnBehaviorPlugin, "/turn_service", "TURN", TurnService);
  };

PLUGINLIB_EXPORT_CLASS(behavior_plugin::TurnBehaviorPlugin, behavior_common::BehaviorPlugin);
