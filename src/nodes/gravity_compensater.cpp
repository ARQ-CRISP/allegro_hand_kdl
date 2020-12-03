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

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Transform.h>
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
double frequency_ = 200; // control freq (Hz)

// state
vector<double> jointstate_;

// base pose transform
tf2_ros::Buffer tf_buffer_;
string hand_frame_="hand_root";
string world_frame_="world";
geometry_msgs::Transform base_tf_;

ros::Publisher pub_; // publishes torques as a JointState msg

// functions
bool getParams();
void updateBasePose();
void timerCallback(const ros::TimerEvent&);
void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg);
void baseposeCallback(const geometry_msgs::Pose::ConstPtr &msg);
void sigintCallback(int sig);

int main(int argc, char **argv)
{

  ros::init(argc, argv, "gravity_compensate");
  ROS_INFO("Allegro gravity compensation");

  ros::NodeHandle nh;

  // attempt to get ros parameters
  if(!getParams()) return -1;

  // create solver
  allegro_kdl_config_.parseKdl(nh);
  dynamics_ = new allegro_hand_kdl::InverseDynamics(allegro_kdl_config_);

  // init state
  jointstate_.resize(FINGER_COUNT*FINGER_LENGTH);

  // create tf listener
  tf2_ros::TransformListener tfListener(tf_buffer_);

  ros::Subscriber sub_js =
      nh.subscribe<sensor_msgs::JointState>( "joint_states", 1, &jointStateCallback, ros::TransportHints().tcpNoDelay().reliable());

  // create publishers
  pub_ =
      nh.advertise<sensor_msgs::JointState>("gravity_compensation_torque", 1);

  // start timer loop
  ros::Timer timer = nh.createTimer(ros::Duration(1/frequency_), timerCallback);

  // ros shutdown handler
  signal(SIGINT, sigintCallback);

  ros::spin();

  delete dynamics_;

  return 0;

}

// get external parameters if possible
bool getParams(){

  string param_name;

  //************
  if(!ros::param::get("/allegro_kdl/kdl_scaler", kdl_scaler_) ){
    ROS_ERROR("Gravity compensate: Can't find kdl_scaler param.");
    return false;
  }
  ROS_DEBUG("Gravity compensate: kdl_scaler is %.3f.", kdl_scaler_);

  //********* hand_frame_
  if(!ros::param::has("~hand_frame") )
    ros::param::get("~hand_frame", hand_frame_);
  ROS_DEBUG( "Gravity compensate: hand base frame: %s", hand_frame_.c_str() );

  //********* world_frame_
  if(ros::param::has("~world_frame") )
    ros::param::get("~world_frame", world_frame_);
  ROS_DEBUG( "Gravity compensate: world base frame: %s", world_frame_.c_str() );

  //********* frequency_
  if(!ros::param::has("~hz") )
    ros::param::get("~hz", frequency_);
  ROS_DEBUG( "Gravity compensate: control frequency: %.2f", frequency_ );

  return true;
}

// update the base pose, compute and publish the torque commands
void timerCallback(const ros::TimerEvent&){

  updateBasePose();

  // Create a JointState msg for torques
  sensor_msgs::JointState msg_out;
  msg_out.header.stamp = ros::Time::now();
  msg_out.position.resize(FINGER_COUNT*FINGER_LENGTH);
  msg_out.velocity.resize(FINGER_COUNT*FINGER_LENGTH);
  msg_out.effort.resize(FINGER_COUNT*FINGER_LENGTH);

  // calculate torques for gravity compensation
  vector<double> q_zeros(FINGER_COUNT*FINGER_LENGTH);

  dynamics_->computeTorques(
      jointstate_, q_zeros, q_zeros, msg_out.effort);

  for (int j=0; j < FINGER_COUNT*FINGER_LENGTH; j++){
    // scale with constant
    msg_out.effort[j] *= kdl_scaler_;

    // names of joints
    msg_out.name.push_back("joint_"+to_string(j));
  }

  // send!
  pub_.publish(msg_out);
}

// read the current joint states
void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg_in) {

  jointstate_ = msg_in->position;
}

void obtainHandTransform(){

  try{
    // transform of hand root w.r.t. robot base
    base_tf_ = tf_buffer_.lookupTransform(
      world_frame_,
      hand_frame_,
      ros::Time(0)).transform;
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
    return;
  }
}

// read the current base frame pose and update the KDL objects
void updateBasePose(){

  obtainHandTransform();

  // update the kdl config
  allegro_kdl_config_.rotateBase(base_tf_.rotation);

  // update the dynamics solvers
  dynamics_->updateChains(allegro_kdl_config_);
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
    msg.name.push_back("joint_"+to_string(j));
  }

  pub_.publish(msg);

  // spin once to handle communication
  ros::spinOnce();

  // All the default sigint handler does is to call shutdown()
  ros::shutdown();
}
