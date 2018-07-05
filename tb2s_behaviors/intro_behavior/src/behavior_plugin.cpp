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
#include <functional>

// NOTE: In its current form, this could just be read as text using the Say behavior.  But this
// behavior is planned to be updated to include motion and camera pan/tilt poses to simulate looking
// around (or using the actual person detector to find and talk to someone specifically)

namespace behavior_plugin 
{
    class IntroBehaviorService : public behavior_common::BehaviorActionServiceBase
    {
      public:
        IntroBehaviorService(std::string service_name) :
          behavior_common::BehaviorActionServiceBase(service_name)
        {
        }

        virtual void StartBehavior(const char *param1, const char *param2)
        {
          if(!speech_.isAvailable())
          {
            ROS_WARN("speech was not available, ending IntroBehaviorService immediately");
            BehaviorComplete(); // Speech not available, so we'll end immediatley
          }

          speech_.speakAndWaitForCompletion("Hello. <break time='500ms'/>");
          speech_.speakAndWaitForCompletion("Well, my name is tee bee two ess.<break time='500ms'/>");
          speech_.speakAndWaitForCompletion("I love humans, other robots, and dogs, but unfortunately dogs dont seem to like me  <break time='1000ms'/>");
        }

        virtual void PremptBehavior()
        {
          // We were requested to preempt our behavior, so cancel any outstanding 
          // speech immediately.
          speech_.cancel();
        }

        protected:
          audio_and_speech_common::SpeechClient speech_;
    };

    CPP_BEHAVIOR_PLUGIN(IntroPlugin, "/intro_behavior_service", "TELL_INTRO", IntroBehaviorService);
  };

PLUGINLIB_EXPORT_CLASS(behavior_plugin::IntroPlugin, behavior_common::BehaviorPlugin);
