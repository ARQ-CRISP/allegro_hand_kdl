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
* This is an action server node for JointPositionController.
* It will run the controller to reach a given action goal,
* publish feedbacks, and return the result when finished.
* Control gains are read from parameters.
**/

#include <signal.h>

#include <ros/ros.h>

#include <allegro_hand_kdl/joint_position_control.h>
#include <allegro_hand_kdl/cartesian_position_control.h>

#include <allegro_hand_kdl/PoseControlAction.h>

#include <actionlib/server/simple_action_server.h>

#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

#include <kdl_control_tools/kdl_helper.h>

using namespace std;
using namespace allegro_hand_kdl;

unique_ptr<actionlib::SimpleActionServer<PoseControlAction> > as_;

unique_ptr<CartesianPositionController> cartesian_control_;
unique_ptr<JointPositionController> joint_control_;

ros::Publisher torque_pub_;

// params
int hz_ = 200;
double safety_torque_ = 1.0;
double time_limit_ = 3.0;

double v_joint_ = 0.5;
double v_cart_ = 0.0;

// state memory
vector<double> q_current_;

void sigintCallback(int sig);
void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg);
void handleAction(const PoseControlGoalConstPtr &goal);
bool getDefinedPoses();
bool readControlGains();
bool getParams();
bool publishTorque(const vector<double>& torque_vec);
void stopMoving();
vector<geometry_msgs::Pose> handPoseKdlToMsg(const HandPose &pose_kdl);
HandPose handPoseMsgToKdl(const vector<geometry_msgs::Pose> &pose_msg);
bool checkFinished(const vector<geometry_msgs::Pose>& p1, const vector<geometry_msgs::Pose>& p2);
bool checkFinished(const vector<double>& p1, const vector<double>& p2);

int main(int argc, char **argv){

  ros::init(argc, argv, "pose_action_server");
  ROS_INFO("Pose action server");

  ros::NodeHandle nh;

  // attempt to get ros parameters
  if(!getParams()) return -1;

  // ros shutdown handler
  signal(SIGINT, sigintCallback);

  // KDL interface objects
  AllegroKdlConfig allegro_config;
  allegro_config.parseKdl(nh);
  cartesian_control_.reset(new CartesianPositionController(allegro_config));
  joint_control_.reset(new JointPositionController());

  // read & set control params
  readControlGains();

  ROS_INFO("Pose action server: started!");

  // create ros communication nodes
  torque_pub_ =
    nh.advertise<sensor_msgs::JointState>("torque", 1);
  ros::Subscriber sub_js =
    nh.subscribe<sensor_msgs::JointState>("joint_states", 1, jointStateCallback, ros::TransportHints().tcpNoDelay().reliable());

  // create an action server
  as_.reset(new actionlib::SimpleActionServer<PoseControlAction>(
      nh, "pose_control_action", boost::bind(&handleAction, _1), false
    ));
  as_->start();

  ros::spin();

  return 0;
}

bool readControlGains(){
  string param_ns = "/allegroHand_right_0/gains";
  if(!ros::param::has(param_ns) ){
    ROS_ERROR("Pose action server: Can't find /allegroHand_right_0/gains param.");
    return false;
  }

  // cartesian control gains
  double kp, kr, kd, ki;
  ros::param::get(param_ns+"/cartesian/pose/p", kp);
  ros::param::get(param_ns+"/cartesian/pose/r", kr);
  ros::param::get(param_ns+"/cartesian/pose/d", kd);
  ros::param::get(param_ns+"/cartesian/pose/i", ki);

  cartesian_control_->setGains(kp, kr, kd, ki);
  cartesian_control_->setIntegralDecay(0.001);

  ROS_DEBUG("Pose action server: KP= %.3f, KR= %.3f, KV= %.3f, KI= %.3f.",
        kp, kr, kd, ki);

  // joint control gains
  vector<double> k_pos_vec(FINGER_LENGTH*FINGER_COUNT);
  vector<double> k_vel_vec(FINGER_LENGTH*FINGER_COUNT);
  vector<double> k_int_vec(FINGER_LENGTH*FINGER_COUNT);

  for(int fi=0; fi < FINGER_COUNT; fi++)
    for(int si=0; si < FINGER_LENGTH; si++){
      ros::param::get(param_ns+"/pose/p/j"+to_string(fi)+to_string(si),
        k_pos_vec[fi*FINGER_LENGTH+si]);
      ros::param::get(param_ns+"/pose/d/j"+to_string(fi)+to_string(si),
        k_vel_vec[fi*FINGER_LENGTH+si]);
      ros::param::get(param_ns+"/pose/i/j"+to_string(fi)+to_string(si),
        k_int_vec[fi*FINGER_LENGTH+si]);
    }

  joint_control_->setGains(k_pos_vec, k_vel_vec, k_int_vec);
  joint_control_->setIntegralDecay(0.001);

  return true;
}

// get external parameters if possible
bool getParams(){

  // safety_torque
  if(!ros::param::get("/allegro_kdl/safety_torque", safety_torque_)){
    ROS_WARN("Pose action server: Can't get safety_torque param. Assuming %.3f.", safety_torque_);
  }
  ROS_DEBUG("Pose action server: safety_torque is %.3f.", safety_torque_);

  if(!ros::param::get("~time_limit", time_limit_)){
    ROS_WARN("Pose action server: Can't get time_limit param. Assuming %.3f.", time_limit_);
  }
  ROS_DEBUG("Pose action server: time_limit is %.3f.", time_limit_);

  return true;
}

void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg) {

  q_current_ = msg->position;
}

void handleAction(const PoseControlGoalConstPtr &goal){

  ros::Rate rate(hz_);
  bool success = false;
  bool err_torque = false;
  bool is_cartesian = false;
  // following are used in case of time limit
  ros::Time t_begin = ros::Time::now();
  ros::Duration limit_duration = ros::Duration(time_limit_);

  if(goal->cartesian_pose.size() > 0) is_cartesian = true;

  ROS_DEBUG("Pose action server: executing action...");

  // loop until completed or preempted
  while(ros::ok() and !err_torque and !as_->isPreemptRequested()){
    rate.sleep();

    // check for the time limit if opted
    if(time_limit_ > 1e-5 && ros::Time::now() - t_begin > limit_duration) break;

    PoseControlFeedback feedback;
    vector<double> torques;

    if(is_cartesian){
      // feedback
      HandPose pose_kdl = cartesian_control_->updatePose(q_current_);
      feedback.cartesian_pose = handPoseKdlToMsg(pose_kdl);
      // control
      vector<geometry_msgs::Twist> xd_des(FINGER_COUNT); // TODO: from average speed
      torques = cartesian_control_->computeTorques(goal->cartesian_pose, xd_des);

      if(checkFinished(goal->cartesian_pose, feedback.cartesian_pose)){
        success = true;
        break;
      }
    }else{
      // feedback
      feedback.joint_pose = q_current_;
      // control
      vector<double> qd_des(FINGER_COUNT*FINGER_LENGTH); // TODO: from average speed
      joint_control_->computeTorques(q_current_, goal->joint_pose, qd_des, torques);

      if(checkFinished(goal->joint_pose, feedback.joint_pose)){
        success = true;
        break;
      }
    }

    err_torque = !publishTorque(torques);
    as_->publishFeedback(feedback);

  }

  if(success){
    ROS_DEBUG("Pose action server: action is successful.");
    PoseControlResult result;
    if(is_cartesian){
      HandPose pose_kdl = cartesian_control_->updatePose(q_current_);
      result.cartesian_pose = handPoseKdlToMsg(pose_kdl);
    }else{
      result.joint_pose = q_current_;
    }

    as_->setSucceeded(result);

  }else{
    ROS_DEBUG("Pose action server: action is preempted.");
    // set the action state to preempted
    as_->setPreempted();
  }

  stopMoving();
  ros::spinOnce();
}

bool publishTorque(const vector<double>& torque_vec){
  // create a joint state message and publish torques
  sensor_msgs::JointState msg;

  int dof = FINGER_COUNT*FINGER_LENGTH;

  msg.header.stamp = ros::Time::now();

  msg.position.resize(dof);
  msg.velocity.resize(dof);

  for (int j=0; j < dof; j++){

    // joint torque is sum of the user input and compensation torques.
    double joint_torque = torque_vec[j];

    // warn user of too much torque
    if(joint_torque > safety_torque_ || joint_torque < -safety_torque_)
    {
      ROS_WARN("Pose action server: Too much torque! %.3f, j: %d", joint_torque, j);
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

  return true;
}

void stopMoving(){
  // send and empty torque message
  vector<double> zeros(FINGER_COUNT*FINGER_LENGTH);
  publishTorque(zeros);
}

vector<geometry_msgs::Pose> handPoseKdlToMsg(const HandPose &pose_kdl){

  vector<geometry_msgs::Pose> pose_arr(FINGER_COUNT);

  for(int i=0; i<FINGER_COUNT; i++)
    tf::poseKDLToMsg(pose_kdl[i], pose_arr[i]);

  return pose_arr;
}

HandPose handPoseMsgToKdl(const vector<geometry_msgs::Pose> &pose_msg){

  HandPose pose_arr(FINGER_COUNT);

  for(int i=0; i<FINGER_COUNT; i++)
    tf::poseMsgToKDL(pose_msg[i], pose_arr[i]);

  return pose_arr;
}

bool checkFinished(const vector<geometry_msgs::Pose>& p1, const vector<geometry_msgs::Pose>& p2){
  double max_dif = 0;
  for(int i=0; i<p1.size(); i++){
    max_dif = max(max_dif, abs(p2[i].position.x-p1[i].position.x));
    max_dif = max(max_dif, abs(p2[i].position.y-p1[i].position.y));
    max_dif = max(max_dif, abs(p2[i].position.z-p1[i].position.z));
  }
  return max_dif < 0.01;
}

bool checkFinished(const vector<double>& p1, const vector<double>& p2){
  double max_dif = 0;
  for(int i=0; i<p1.size(); i++)
    max_dif = max(max_dif, abs(p2[i]-p1[i]));
  return max_dif < 0.1;
}


// This procedure is called when the ros node is interrupted
void sigintCallback(int sig){

  stopMoving();

  // spin once to hand communication
  ros::spinOnce();

  // All the default sigint handler does is to call shutdown()
  ros::shutdown();
}
