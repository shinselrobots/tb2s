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
#include <audio_and_speech_common/audio_and_speech_common.h>
#include <std_msgs/Float64.h>
#include <functional>
#include <time.h>

static std::vector<std::string> ack_text = {
  "Lets see", 
  "well, lets see", 
  "let me check", 
  "sure, let me look at my watch" 
};

static std::vector<std::string> final_text = {
  "Was that really a good use of 2.6 billion transistors on a Skylake processor?", 
  "I have 5 thousand lines of code, so you can ask me the time?  I'm sure that was a good investment.",
  "My watch is synchronized with the atomic clock in Boulder, Colorado",
  "Did that help?",
  "I think perhaps I am the world's most expensive clock"
};

namespace behavior_plugin 
{
    class TimeBehaviorService : public behavior_common::BehaviorActionServiceBase
    {
      public:
        TimeBehaviorService(std::string service_name) :
          behavior_common::BehaviorActionServiceBase(service_name)
        {
          s1_.set(ack_text);
          s2_.set(final_text);

          // Publish robot camera movement
          pan_ = nh_.advertise<std_msgs::Float64>("/head_pan_controller/command", 1); 
          tilt_ = nh_.advertise<std_msgs::Float64>("/head_tilt_controller/command", 1);
        }

        virtual void StartBehavior(const char *param1, const char *param2)
        {
          std_msgs::Float64 pos;

          if(!speech_.isAvailable())
          {
            ROS_WARN("speech was not available, ending TimeBehaviorService immediately");
            BehaviorComplete(); // Speech not available, so we'll end immediatley
          }

          time_t rawtime;
          struct tm * timeinfo;
          std::ostringstream stream;

          speech_.speakAndWaitForCompletion(s1_.shuffle_next());
       
          // Simulate delay to read time   
          pos.data = 0.0;
          pan_.publish(pos);
          pos.data = 0.65;
          tilt_.publish(pos);

          boost::this_thread::sleep(boost::posix_time::milliseconds(2000));

          pos.data = -0.5;
          tilt_.publish(pos);


          // Determine the actual time and say it
          time ( &rawtime );
          timeinfo = localtime ( &rawtime );

          stream << "The time is ";
          if( timeinfo->tm_hour > 12 )
            stream << (timeinfo->tm_hour - 12) << " " << timeinfo->tm_min << " PM";
          else
            stream << timeinfo->tm_hour << " " << timeinfo->tm_min << " AM";
          speech_.speakAndWaitForCompletion(stream.str());

          // Finalize behavior
          speech_.speakAndWaitForCompletion(s2_.shuffle_next());

          pos.data = 0.0;
          tilt_.publish(pos);
        }

        virtual void PremptBehavior()
        {
          // We were requested to preempt our behavior, so cancel any outstanding 
          // speech immediately.
          speech_.cancel();
        }

        protected:
          ros::NodeHandle nh_;
          audio_and_speech_common::SpeechClient speech_;
          audio_and_speech_common::speech_indexer s1_, s2_;
          ros::Publisher pan_;
          ros::Publisher tilt_;
    };

    CPP_BEHAVIOR_PLUGIN(TimePlugin, "/time_behavior_service", "TELL_TIME", TimeBehaviorService);
  };

PLUGINLIB_EXPORT_CLASS(behavior_plugin::TimePlugin, behavior_common::BehaviorPlugin);
