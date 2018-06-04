/*
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <ros/ros.h>
#include <turtlebot3_safety.h>

//==============================================================================
// Constructor
//==============================================================================
Turtlebot3Safety::Turtlebot3Safety(void) 
{
  state_changed_ = false;
  cliff_left_detected_ = false;
  cliff_front_detected_ = false;
  cliff_right_detected_ = false;
  cliff_rear_detected_ = false;
  bumper_left_detected_ = false;
  bumper_right_detected_ = false;

  // Wonder if this should be two step init...
  cliff_event_subscriber_  = nh_.subscribe<turtlebot3_msgs::CliffEvent>("events/cliff",  10, &Turtlebot3Safety::cliffEventCB, this);
  bumper_event_subscriber_  = nh_.subscribe<turtlebot3_msgs::BumperEvent>("events/bumper",  10, &Turtlebot3Safety::bumperEventCB, this);
  velocity_command_publisher_ = nh_.advertise< geometry_msgs::Twist >("cmd_vel", 10);
}

//==============================================================================
// processEvents
//==============================================================================
void Turtlebot3Safety::processEvents()
{
  if (state_changed_) {
    ROS_INFO("Turtlebot3Safety process Events");
    // Not sure if we output once or if we should continue to output...
    if (cliff_front_detected_) {
      publishVelocityCmd(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);    // All stop
    } else if (cliff_left_detected_) {
      publishVelocityCmd(-0.1, 0.0, 0.0, 0.0, 0.0, -0.4);  // Turn a little to the right
    } else if (cliff_right_detected_) {
      publishVelocityCmd(-0.1, 0.0, 0.0, 0.0, 0.0, 0.4);  // Turn a little to the left
    }
    state_changed_ = false;
  }

}

//==============================================================================
// publishVelocityCmd - helper function to output a new command velocity
//==============================================================================
void Turtlebot3Safety::publishVelocityCmd(double lx, double ly, double lz, double ax, double ay, double az) 
{
  cmd_vel_.linear.x = lx;
  cmd_vel_.linear.y = ly;
  cmd_vel_.linear.z = lz;
  cmd_vel_.angular.x = ax;
  cmd_vel_.angular.y = ay;
  cmd_vel_.angular.z = az;
  velocity_command_publisher_.publish(cmd_vel_);
}


//==============================================================================
// CliffCallback...
//==============================================================================
void Turtlebot3Safety::cliffEventCB(const turtlebot3_msgs::CliffEvent::ConstPtr &msg) 
{
  ROS_INFO("CliffEvent: sensor:%x state:%x bottom: %u", msg->sensor, msg->state, msg->bottom);
  // Lets see what type of event:
  if (msg->state == turtlebot3_msgs::CliffEvent::CLIFF) {
    switch (msg->sensor) {
      case turtlebot3_msgs::CliffEvent::CLIFF_LEFT:   cliff_left_detected_ = true;   break;
      case turtlebot3_msgs::CliffEvent::CLIFF_CENTER: cliff_front_detected_ = true;  break;
      case turtlebot3_msgs::CliffEvent::CLIFF_RIGHT:  cliff_right_detected_ = true;  break;
      case turtlebot3_msgs::CliffEvent::CLIFF_REAR:   cliff_rear_detected_ = true;   break;
    }
  } else {
    switch (msg->sensor) {
      case turtlebot3_msgs::CliffEvent::CLIFF_LEFT:   cliff_left_detected_ = false;   break;
      case turtlebot3_msgs::CliffEvent::CLIFF_CENTER: cliff_front_detected_ = false;  break;
      case turtlebot3_msgs::CliffEvent::CLIFF_RIGHT:  cliff_right_detected_ = false;  break;
      case turtlebot3_msgs::CliffEvent::CLIFF_REAR:   cliff_rear_detected_ = false;   break;
    }
  }    
  state_changed_ = true;
}

//==============================================================================
// CliffCallback...
//==============================================================================
void Turtlebot3Safety::bumperEventCB(const turtlebot3_msgs::BumperEvent::ConstPtr &msg) 
{
  ROS_INFO("BumperEvent: bumper:%x state:%x", msg->bumper, msg->state);
  // Lets see what type of event:
  if (msg->state == turtlebot3_msgs::BumperEvent::PRESSED) {
    switch (msg->bumper) {
      case turtlebot3_msgs::BumperEvent::LEFT_BUMPER:  bumper_left_detected_ = true;   break;
      case turtlebot3_msgs::BumperEvent::RIGHT_BUMPER: bumper_right_detected_ = true;  break;
    }
  } else {
    switch (msg->bumper) {
      case turtlebot3_msgs::BumperEvent::LEFT_BUMPER:  bumper_left_detected_ = false;   break;
      case turtlebot3_msgs::BumperEvent::RIGHT_BUMPER: bumper_right_detected_ = false;  break;
    }
  }    
  state_changed_ = true;
}



//==============================================================================
// Main
//==============================================================================
// Pretty simple main function, We simply setup to subsribe to the cliff event and 
// Soon the bumper events.  When the events happen, we send off messages to try to either stop or
// maybe change the direction of the turtle...
int main(int argc, char **argv)
{
  ros::init(argc, argv, "turtlebot3_safety");

  Turtlebot3Safety turtlebot3Safety;  // define instance of it in our main line. 

  ros::AsyncSpinner spinner(1); // Using 1 threads
  spinner.start();

  ros::Rate loop_rate( 100 ); // 100 hz

  while ( ros::ok() )
  {
    // See if we need to output anything...
    turtlebot3Safety.processEvents();
    loop_rate.sleep();
  }
	return 0;
}

