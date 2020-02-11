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


#include <allegro_hand_kdl/kinematics.h>

#include <kdl_conversions/kdl_msg.h>

#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/JointState.h>

/**
* This node will listen to allegro hand JointState
* and publish fingertip poses as PoseArray
**/

using namespace std;
using namespace allegro_hand_kdl;
using namespace KDL;

AllegroKdlConfig allegro_kdl_config_;
Kinematics* kinematics_;

ros::Publisher pub_; // publish fingertip frames as a PoseArray

void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg_in) {

  // update the state
  vector<double> joint_pos_vec = msg_in->position;

  // apply forward kinematics
  vector<KDL::Frame> frames_kdl;
  kinematics_->calcCartPos(joint_pos_vec, frames_kdl);

  // create geometry_msgs::PoseArray
  geometry_msgs::PoseArray msg_out;

  msg_out.header.stamp = ros::Time::now();
  msg_out.poses.resize(FINGER_COUNT);

  // convert from kdl to geometry_msgs
  for(int fi=0; fi<FINGER_COUNT; fi++)
    tf::poseKDLToMsg(frames_kdl[fi], msg_out.poses[fi]);

  // send!
  pub_.publish(msg_out);

}

bool getJointLimits(ros::NodeHandle& nh, vector<double>& qmin, vector<double>& qmax){

  qmin.resize(FINGER_LENGTH*FINGER_COUNT);
  qmax.resize(FINGER_LENGTH*FINGER_COUNT);

  string param_name;
  if(!nh.searchParam("/allegroHand_right_0/joint_limits", param_name) ){
    ROS_ERROR("Allegro FK: Can't find joint_limits param.");
    return false;
  }

  for(int fi=0; fi < FINGER_COUNT; fi++)
    for(int si=0; si < FINGER_LENGTH; si++){
      ros::param::get(param_name+"/min/j"+to_string(fi)+to_string(si),
        qmin[fi*FINGER_LENGTH+si]);
      ros::param::get(param_name+"/max/j"+to_string(fi)+to_string(si),
        qmax[fi*FINGER_LENGTH+si]);
    }

  return true;
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "fk_publisher");
  ROS_INFO("Allegro FK node");

  ros::NodeHandle nh;

  // create solver
  allegro_kdl_config_.parseKdl(nh);
  kinematics_ = new Kinematics(allegro_kdl_config_);

  ros::Subscriber sub = nh.subscribe<sensor_msgs::JointState>( "joint_states", 1, &jointStateCallback, ros::TransportHints().tcpNoDelay().reliable());

  // create publishers
  pub_ = nh.advertise<geometry_msgs::PoseArray>("pose_array", 1);

  ros::spin();

  delete kinematics_;

  return 0;

}
