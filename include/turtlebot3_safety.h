// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of the <organization> nor the
//     names of its contributors may be used to endorse or promote products
//     derived from this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef TURTLEBOT3_SAFETY_H_
#define TURTLEBOT3_SAFETY_H_
#include <ros/ros.h>
#include <ros/types.h>
#include <turtlebot3_msgs/CliffEvent.h>
#include <turtlebot3_msgs/BumperEvent.h>
#include <geometry_msgs/Twist.h>

class Turtlebot3Safety 
{
	public:
		Turtlebot3Safety (void);
		void processEvents();

	    geometry_msgs::Twist cmd_vel_;

	    bool state_changed_;
		bool cliff_left_detected_;
		bool cliff_front_detected_;
		bool cliff_right_detected_;
		bool cliff_rear_detected_;
		bool bumper_left_detected_;
		bool bumper_right_detected_;

private:
		ros::NodeHandle nh_;
		ros::Subscriber cliff_event_subscriber_;
		ros::Subscriber bumper_event_subscriber_;
		ros::Publisher velocity_command_publisher_;
		void cliffEventCB(const turtlebot3_msgs::CliffEvent::ConstPtr &msg);
		void bumperEventCB(const turtlebot3_msgs::BumperEvent::ConstPtr &msg);
		void publishVelocityCmd(double lx, double ly, double lz, double ax, double ay, double az);
};

#endif // TURTLEBOT3_SAFETY_H_