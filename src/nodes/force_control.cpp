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

/*********************************************************************
* Force Control Node calculates the joint torques needed to
* apply desired forces and publishes them to control the Allegro Hand.
* It subscibes to a cartesian force topic to get the desired forces.
* It also subscribes to a JointState topic as this info is needed
* to calculate the current Jacobian matrix which is used to calculate
* the needed torques.
*********************************************************************/

#include <signal.h>

#include <allegro_hand_kdl/cartesian_force.h>

#include <kdl_conversions/kdl_msg.h>

#include <kdl_control_tools/WrenchArray.h>
#include <sensor_msgs/JointState.h>

using namespace std;
using namespace KDL;
using namespace allegro_hand_kdl;

const int WRENCH_SIZE = 6;

CartesianForce* cf_solver_;

ros::Publisher torque_pub_;

// states
vector<double> joint_pos_vec_;
vector<vector<double> > wrench_vec_;

// control params
double safety_torque_;
double update_freq_ = 200;

// desired forces coming from the subscribed topic
// vector<double> f_des_;

// functions
bool getParams(ros::NodeHandle nh);
void cartForceCallback(const kdl_control_tools::WrenchArray::ConstPtr &msg_forces);
void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg);
void updateCallback(const ros::TimerEvent&);
void publishTorques(const vector<double> &tau);
void sigintCallback(int sig);

int main(int argc, char **argv)
{

  ros::init(argc, argv, "force_control_node");
  ROS_INFO("Allegro Force Control Node");

  ros::NodeHandle nh;

  // attempt to get ros parameters
  if(!getParams(nh)) return -1;

  // KDL interface objects
  allegro_hand_kdl::AllegroKdlConfig allegro_config;
  allegro_config.parseKdl(nh);
  cf_solver_ = new CartesianForce(allegro_config);

  // ros communication
  ros::Subscriber sub_js =
      nh.subscribe<sensor_msgs::JointState>( "joint_states", 1, &jointStateCallback);
  ros::Subscriber sub_cf =
      nh.subscribe<kdl_control_tools::WrenchArray>("cartesian_forces", 1, &cartForceCallback);
  torque_pub_ =
      nh.advertise<sensor_msgs::JointState>("allegro_torque", 1);

  // timer callback for the main loop
  ros::Timer timer = nh.createTimer(ros::Duration(1/update_freq_), &updateCallback);

  // ros shutdown handler
  signal(SIGINT, sigintCallback);

  ros::spin();

  delete cf_solver_;

  return 0;

}

// get external parameters if possible
bool getParams(ros::NodeHandle nh){

  string param_name;

  if(!nh.searchParam("/allegro_kdl/safety_torque", param_name) ){
    ROS_ERROR("Force Control: Can't find safety_torque param.");
    return false;
  }
  ros::param::get(param_name, safety_torque_);
  ROS_DEBUG("Force Control: safety torque is %.3f.", safety_torque_);

  return true;
}

/**
  * Main update loop
 **/
void updateCallback(const ros::TimerEvent&){

  // wait until getting a proper joint position vector
  if(joint_pos_vec_.size() != FINGER_COUNT*FINGER_LENGTH || wrench_vec_.size() != FINGER_COUNT) return;

  // calculate the joint torques needed to achieve finger cart forces
  vector<double> torque_joint(FINGER_COUNT*FINGER_LENGTH);
  torque_joint = cf_solver_->computeTorques(joint_pos_vec_, wrench_vec_);

  for(int j=0; j<16; j++)
    ROS_DEBUG("CartesianForce, torque %d, %f", j, torque_joint[j]);

  // publish the joint torque commands
  publishTorques(torque_joint);
}

// create a joint state message and publish torques
void publishTorques(const vector<double> &tau){

  // create the message and its header
  sensor_msgs::JointState msg;
  msg.header.stamp = ros::Time::now();

  // message size
  int dim = tau.size();

  msg.position.resize(dim);
  msg.velocity.resize(dim);

  for (int j=0; j < dim; j++){

    // scale
    double joint_torque = tau[j] * 1.0;

    // warn user of too much torque
    if(joint_torque > safety_torque_ || joint_torque < -safety_torque_)
    {
      ROS_WARN("Force control: Too much torque! %.3f, j: %d", joint_torque, j);
    }

    // shave torque instead of emergency stop
    joint_torque = max(joint_torque, -safety_torque_);
    joint_torque = min(joint_torque, safety_torque_);

    // add torques to message as effort
    msg.effort.push_back(joint_torque);
    // names of joints
    msg.name.push_back("joint_"+to_string(j));
  }

  torque_pub_.publish(msg);
}

// Listens to cartesian forces published by another node,
// publishes the joint torques to achieve these forces.
void cartForceCallback(const kdl_control_tools::WrenchArray::ConstPtr &msg_forces) {

    // Calculate joint torques for cartesian force
  // create a wrench vec for each finger
  wrench_vec_.resize(FINGER_COUNT);
  for(int fi=0; fi < FINGER_COUNT; fi++){

    wrench_vec_[fi].resize(6, 0.0);

    // add to wrench vector to publish
    wrench_vec_[fi][0] = msg_forces->wrenches[fi].force.x;
    wrench_vec_[fi][1] = msg_forces->wrenches[fi].force.y;
    wrench_vec_[fi][2] = msg_forces->wrenches[fi].force.z;
    wrench_vec_[fi][3] = msg_forces->wrenches[fi].torque.x;
    wrench_vec_[fi][4] = msg_forces->wrenches[fi].torque.y;
    wrench_vec_[fi][5] = msg_forces->wrenches[fi].torque.z;
  }

}

// updates the joint states
void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg) {
  // update the state
  joint_pos_vec_ = msg->position;
}


// This procedure is called when the ros node is interrupted
void sigintCallback(int sig)
{
  // send and empty torque message
  sensor_msgs::JointState msg;

  msg.header.stamp = ros::Time::now();

  msg.position.resize(16);
  msg.velocity.resize(16);
  msg.effort.resize(16);

  for (int j=0; j < 16; j++){
    msg.name.push_back("joint_"+to_string(j));
  }

  torque_pub_.publish(msg);

  // spin once to handle communication
  ros::spinOnce();

  // All the default sigint handler does is to call shutdown()
  ros::shutdown();
}
