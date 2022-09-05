/**
* allegro_hand_kdl is a ROS package to control Allegro Hand using the KDL library.
* Copyright (C) 2020  Gokhan Solak, CRISP, Queen Mary University of London
*
* This library is free software; you can redistribute it and/or
* modify it under the terms of the GNU Lesser General Public
* License as published by the Free Software Foundation; either
* version 2.1 of the License, or (at your option) any later version.
*
* This library is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License along with this library; if not, write to the Free Software
* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301
* USA
*/


#include <ros/ros.h>

#include <allegro_hand_kdl/pose_control_client.h>

#define PI 3.14159265

/**
* This node creates a PoseControlClient and commands
* the server by creating new poses.
**/

using namespace std;
using namespace allegro_hand_kdl;

shared_ptr<CartesianPoseClient> cartesian_client;
shared_ptr<JointPoseClient> joint_client;

vector<geometry_msgs::Pose> cartesian_pose;
vector<double> joint_pose;

double phase;

void timerCallback(const ros::TimerEvent&);

int main(int argc, char **argv)
{

  ros::init(argc, argv, "pose_client");
  ROS_INFO("Pose Client Node");

  ros::NodeHandle nh;

  // create the clients
  cartesian_client =
    make_shared<CartesianPoseClient>();
  joint_client =
    make_shared<JointPoseClient>();

  // TODO: listen to topics for continuous pose requests

  ROS_INFO("Moving the hand for testing");

  // init control variables
  phase = 0;
  ros::param::get("allegro_kdl/poses/joint_poses/p1/state", joint_pose); // relax pose

  // start timer loop
  ros::Timer timer = nh.createTimer(ros::Duration(0.3), timerCallback);

  ros::spin();

  return 0;

}

void timerCallback(const ros::TimerEvent&){
  // for testing, move the fingers in a basic motion
  vector<double> new_pose = joint_pose;

  for(int ji=0; ji<FINGER_COUNT*FINGER_LENGTH; ji++){
    if(ji%4==0)
      continue; // don't move the first joints
    if(ji%3==0)
      new_pose[ji] += sin(phase) * 0.3;
    else
      new_pose[ji] -= sin(phase) * 0.3;
  }

  // usage of the client object
  joint_client->setTargetPose(new_pose, "test");
  joint_client->move();

  // increment phase
  phase += PI / 8;
  phase = fmod(phase, PI);
}
