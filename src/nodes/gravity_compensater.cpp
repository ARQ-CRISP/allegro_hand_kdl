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

#include <signal.h>

#include <allegro_hand_kdl/inverse_dynamics.h>

#include <kdl_conversions/kdl_msg.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>

/**
* This node will publish torque commands to compensate gravity
* in its current state. It listens to allegro hand joint states
* and base frame pose to read the current state.
**/

using namespace std;
using namespace allegro_hand_kdl;
using namespace KDL;

AllegroKdlConfig allegro_kdl_config_;
InverseDynamics* dynamics_;

// control params
double kdl_scaler_;

ros::Publisher pub_; // publishes torques as a JointState msg

// functions
bool getParams(ros::NodeHandle nh);
void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg);
void baseposeCallback(const geometry_msgs::Pose::ConstPtr &msg);
void sigintCallback(int sig);

int main(int argc, char **argv)
{

  ros::init(argc, argv, "gravity_compensate");
  ROS_INFO("Allegro gravity compensation");

  ros::NodeHandle nh;

  // attempt to get ros parameters
  if(!getParams(nh)) return -1;

  // create solver
  allegro_kdl_config_.parseKdl(nh);
  dynamics_ = new allegro_hand_kdl::InverseDynamics(allegro_kdl_config_);


  ros::Subscriber sub_js =
      nh.subscribe<sensor_msgs::JointState>( "joint_states", 1, &jointStateCallback, ros::TransportHints().tcpNoDelay().reliable());

  ros::Subscriber sub_pose =
      nh.subscribe<geometry_msgs::Pose>( "base_pose", 1, &baseposeCallback, ros::TransportHints().tcpNoDelay().reliable());

  // create publishers
  pub_ =
      nh.advertise<sensor_msgs::JointState>("gravity_compensation_torque", 1);

  // ros shutdown handler
  signal(SIGINT, sigintCallback);

  ros::spin();

  delete dynamics_;

  return 0;

}

// get external parameters if possible
bool getParams(ros::NodeHandle nh){

  string param_name;

  //************
  if(!nh.searchParam("/allegro_kdl/kdl_scaler", param_name) ){
    ROS_ERROR("Gravity compensate: Can't find kdl_scaler param.");
    return false;
  }
  ros::param::get(param_name, kdl_scaler_);
  ROS_DEBUG("Gravity compensate: kdl_scaler is %.3f.", kdl_scaler_);

  return true;
}

// read the current base frame pose and update the KDL objects
void baseposeCallback(const geometry_msgs::Pose::ConstPtr &msg){

  // update the kdl config
  allegro_kdl_config_.rotateBase(msg->orientation);

  // update the dynamics solvers
  dynamics_->updateChains(allegro_kdl_config_);

}

// read the current joint states and publish the torque commands
void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg_in) {

  // update the state
  vector<double> joint_pos_vec = msg_in->position;

  // Create a JointState msg for torques
  sensor_msgs::JointState msg_out;
  msg_out.header.stamp = ros::Time::now();
  msg_out.position.resize(FINGER_COUNT*FINGER_LENGTH);
  msg_out.velocity.resize(FINGER_COUNT*FINGER_LENGTH);
  msg_out.effort.resize(FINGER_COUNT*FINGER_LENGTH);

  // calculate torques for gravity compensation
  vector<double> q_zeros(FINGER_COUNT*FINGER_LENGTH);

  dynamics_->computeTorques(
      joint_pos_vec, q_zeros, q_zeros, msg_out.effort);

  for (int j=0; j < FINGER_COUNT*FINGER_LENGTH; j++){
    // scale with constant
    msg_out.effort[j] *= kdl_scaler_;

    // names of joints
    msg_out.name.push_back("joint_"+to_string(j+1));
  }

  // send!
  pub_.publish(msg_out);

}

// This procedure is called when the ros node is interrupted
void sigintCallback(int sig){
  // send and empty torque message
  sensor_msgs::JointState msg;

  msg.header.stamp = ros::Time::now();

  msg.position.resize(FINGER_COUNT*FINGER_LENGTH);
  msg.velocity.resize(FINGER_COUNT*FINGER_LENGTH);
  msg.effort.resize(FINGER_COUNT*FINGER_LENGTH);

  for (int j=0; j < FINGER_COUNT*FINGER_LENGTH; j++){
    msg.name.push_back("joint_"+to_string(j+1));
  }

  pub_.publish(msg);

  // spin once to handle communication
  ros::spinOnce();

  // All the default sigint handler does is to call shutdown()
  ros::shutdown();
}
