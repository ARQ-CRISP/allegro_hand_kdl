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

/**
* Reads the torque cmd messages and chanes Allegro's
* joint states to create a fake movement for simulation.
**/

#include <ros/ros.h>

#include <sensor_msgs/JointState.h>

using namespace std;

ros::Publisher state_pub_;
sensor_msgs::JointState last_state_;

vector<vector<double> > joint_limits_;

ros::Time t_update_;
double move_ratio_ = 5.0;

// hand id
string hand_side_;
int hand_no_;

bool getJointLimits(){

  // get hand id, return error if not given
  if(!ros::param::get("~hand", hand_side_)) return false;
  if(!ros::param::get("~num", hand_no_)) return false;

  // create joint limits
  joint_limits_.resize(16);

  // check if limits defined
  string param_ns = "/allegroHand_"+hand_side_+"_"+to_string(hand_no_)+"/joint_limits";
  if(!ros::param::has(param_ns)){

    ROS_WARN("Fake control: Can't get limit params. No limits!");

    for(int ji = 0; ji<16; ji++)
      joint_limits_[ji].resize(2);

  }else{ // if defined
    // read limits for each segment
    for(int fi = 0; fi<4; fi++)
        for(int si = 0; si<4; si++){
          joint_limits_[fi*4+si].resize(2);
          ros::param::get(param_ns+"/min/j"+to_string(fi)+to_string(si), joint_limits_[fi*4+si][0]);
          ros::param::get(param_ns+"/max/j"+to_string(fi)+to_string(si), joint_limits_[fi*4+si][1]);
        }
  }

  return true;
}

double moveLimited(int index, double state, double move){
  double lower_limit = joint_limits_[index][0];
  double upper_limit = joint_limits_[index][1];

  double limited_move = state + move;

  // apply upper limit
  limited_move = min(limited_move, upper_limit);
  // apply lower limit
  limited_move = max(limited_move, lower_limit);

  // return the saturated value
  return limited_move;
}

void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg) {

  last_state_.position = msg->position;
  last_state_.velocity.resize(16);
  last_state_.effort.resize(16);

}

void torqueCmdCallback(const sensor_msgs::JointState::ConstPtr &msg) {

  if(last_state_.position.size() == 0) return;

  // Calc time since last call
  ros::Time tnow_ = ros::Time::now();
  double dt_update = (tnow_ - t_update_).sec + 1e-9 * (tnow_ - t_update_).nsec;

  bool moved = false;

  // update each joint position wrt to torque command and delta time
  for(int ji = 0; ji<16; ji++){
    double move = dt_update * move_ratio_ * msg->effort[ji];

    last_state_.position[ji] = moveLimited(ji, last_state_.position[ji], move);

    if(msg->effort[ji] > 0) moved = true;
  }

  // timing
  t_update_ = ros::Time::now();
  last_state_.header.stamp = t_update_;

  // publish new state
  if(moved) state_pub_.publish(last_state_);

}


int main(int argc, char **argv){

  ros::init(argc, argv, "fake_control");
  ROS_INFO("Allegro Fake Controller node");

  ros::NodeHandle nh;

  // try to read joint limits from params
  if(!getJointLimits()) return -1;

  // timer
  t_update_ = ros::Time::now();

  state_pub_ =
    nh.advertise<sensor_msgs::JointState>("joint_cmd", 1);
  ros::Subscriber sub_js =
    nh.subscribe<sensor_msgs::JointState>("joint_states", 1, jointStateCallback);
  ros::Subscriber sub_torque =
    nh.subscribe<sensor_msgs::JointState>("torque_cmd", 1, torqueCmdCallback);

  ros::spin();

  return 0;

}
