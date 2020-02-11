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
* This is a server node for cartesian force control.
* If a PoseRequest service is issued, then the fingers are moved
* to the predefined pose states.
* Pose states and control gains are read from parameters.
**/
#include <signal.h>
#include <memory>

#include <allegro_hand_kdl/cartesian_position_control.h>
#include <allegro_hand_kdl/PoseRequest.h>

#include <kdl_control_tools/progress_logger.h>

#include <kdl_conversions/kdl_msg.h>
#include <tf2/LinearMath/Quaternion.h>

#include <sensor_msgs/JointState.h>

using namespace std;
using namespace allegro_hand_kdl;

// A HandPose is a vector<KDL::Frame>
typedef shared_ptr<HandPose> HandPosePtr;

unique_ptr<CartesianPositionController> cp_control_;

ros::Publisher torque_pub_; // to control the robot

// desired average speed
double v_lim_vel_ = 0.03; // meters per second
double v_lim_rot_ = 0.35; // radians per second

// params
double period_ = 0.005;
double safety_torque_ = 1.0;
bool maintain_ = true;
double stop_torque_ = 0.06;
double time_limit_ = 4.0;
string pose_param_ns_;

double k_p_ = 10;
double k_r_ = 1;
double k_d_ = 0.001;
double k_i_ = 0.0;

// state memory
vector<double> q_current_;
HandPose x_des_;

ros::Time t_update_;
ros::Time t_begin_;

bool success_ = false;

bool state_read_ = false;
bool finished_ = true;

void sigintCallback(int sig);
void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg);
void timerCallback(const ros::TimerEvent&);
bool processPoseRequest(allegro_hand_kdl::PoseRequest::Request& req, allegro_hand_kdl::PoseRequest::Response& res);
HandVelocity limitedDesiredVelocity(const HandPose& x_cur);
vector<KDL::Frame> getDefinedPose(string desired_name);
bool getParams();

int main(int argc, char **argv){

  ros::init(argc, argv, "cartesian_pose_server");
  ROS_INFO("Cartesian pose server node");

  ros::NodeHandle nh;

  // attempt to get ros parameters
  if(!getParams()) return -1;

  // ros shutdown handler
  signal(SIGINT, sigintCallback);

  // KDL interface objects
  AllegroKdlConfig allegro_config;
  allegro_config.parseKdl(nh);
  cp_control_.reset(new CartesianPositionController(allegro_config));

  // set cartesian position control params
  cp_control_->setGains(k_p_, k_r_, k_d_, k_i_);
  cp_control_->setIntegralDecay(0.001);

  // countdown only if there is an initial pose
  if(x_des_.size() == FINGER_COUNT){
    finished_ = false;

    ros::Duration sleep_time(1.0);

    for (int ci = 3; ci > 0; ci--){
      ROS_INFO( "Cartesian Pose Server: starting in %d", ci);
      sleep_time.sleep();
    }
  }
  ROS_INFO("Cartesian Pose Server: started!");

  // create ros communication nodes
  torque_pub_ =
    nh.advertise<sensor_msgs::JointState>("position_torque", 1);
  ros::Subscriber sub_js =
    nh.subscribe<sensor_msgs::JointState>("joint_states", 1, jointStateCallback, ros::TransportHints().tcpNoDelay().reliable());

  ros::ServiceServer service =
    nh.advertiseService("desired_cartesian_pose", processPoseRequest);

  // start the control loop
  t_begin_ = ros::Time::now();
  ros::Timer timer = nh.createTimer(ros::Duration(period_), &timerCallback);

  ros::spin();

  return 0;

}

bool getControlGains(){

  string param_name = "/allegroHand_right_0/gains/cartesian/pose";
  if(!ros::param::has(param_name) ){
    ROS_ERROR("Cartesian Pose Server: Can't find gains/cartesian/pose param.");
    return false;
  }

  ros::param::get(param_name+"/p", k_p_);
  ros::param::get(param_name+"/r", k_r_);
  ros::param::get(param_name+"/d", k_d_);
  ros::param::get(param_name+"/i", k_i_);

  ROS_DEBUG("Cartesian Pose Server: KP= %.3f, KR= %.3f, KV= %.3f, KI= %.3f.",
        k_p_, k_r_, k_d_, k_i_);

  return true;
}

// get external parameters if possible
bool getParams(){

  // safety_torque
  if(!ros::param::get("/allegro_kdl/safety_torque", safety_torque_)){
    ROS_WARN("Cartesian Pose Server: Can't get safety_torque param. Assuming %.3f.", safety_torque_);
  }
  ROS_DEBUG("Cartesian Pose Server: safety_torque is %.3f.", safety_torque_);

  // control params
  getControlGains();

  // time limit
  if(!ros::param::get("~time_limit", time_limit_)){
    ROS_WARN("Cartesian Pose Server: Can't get time limit param.");
  }

  // get the pose param namespace param if specified
  if(!ros::param::get("~pose_ns", pose_param_ns_)){
    pose_param_ns_ = "allegro_kdl/poses";
    ROS_INFO("Cartesian Pose Server: Assuming default namespace %s.", pose_param_ns_.c_str());
  }
  pose_param_ns_ += "/cartesian_poses";
  // make sure that the params exist
  if(!ros::param::has(pose_param_ns_)){
    ROS_ERROR("Cartesian Pose Server: Can't find poses %s", pose_param_ns_.c_str());
    return false;
  }

  // maintain pose
  if(!ros::param::get("~maintain_pose", maintain_)){
    ROS_WARN("Cartesian Pose Server: Can't get maintain_pose param.");
  }
  if (maintain_){
    // activate controller if maintain
    finished_ = false;
    ROS_INFO("Cartesian Pose Server: maintaining pose.");
  }
  // maintain pose
  string str_pose;
  if(ros::param::get("~initial_pose", str_pose)){
    // get the pose if exists
    vector<KDL::Frame> pose = getDefinedPose(str_pose);
    if(pose.size() == FINGER_COUNT){
      // assign the pose and activate controller
      x_des_ = pose;
      finished_ = false;
      // inform
      ROS_INFO("Cartesian Pose Server: Initial pose is %s.", str_pose.c_str());
    }
  }

  return true;
}

// get pose from server whenever necessary
vector<KDL::Frame> getDefinedPose(string desired_name){

  // iterate existing poses (following pattern p0,p1...) until finding the one
  int pi = 0;
  while(ros::param::has(pose_param_ns_+"/p"+to_string(pi))){

    string name;
    ros::param::get(pose_param_ns_+"/p"+to_string(pi)+"/name", name);

    // skip until match
    if(name != desired_name){
      pi++;
      continue;
    }
    // create fingertip poses for this name
    vector<KDL::Frame> pose(FINGER_COUNT);

    // read states for each finger
    for(int fi=0; fi<FINGER_COUNT; fi++){

      string pose_path =
          pose_param_ns_+"/p"+to_string(pi)+"/state/f"+to_string(fi);
      // position
      ros::param::get(pose_path+"/pos/x", pose[fi].p.data[0]);
      ros::param::get(pose_path+"/pos/y", pose[fi].p.data[1]);
      ros::param::get(pose_path+"/pos/z", pose[fi].p.data[2]);
      // orientation
      double quat_x,quat_y,quat_z,quat_w;
      ros::param::get(pose_path+"/rot/x", quat_x);
      ros::param::get(pose_path+"/rot/y", quat_y);
      ros::param::get(pose_path+"/rot/z", quat_z);
      ros::param::get(pose_path+"/rot/w", quat_w);
        // create Quaternion from Euler YXZ
      pose[fi].M = KDL::Rotation::Quaternion(quat_x, quat_y, quat_z, quat_w);
    }

    // if match method ends here
    return pose;
  }

  ROS_ERROR("Cartesian Pose Server: can't find the pose %s", desired_name.c_str());
  return vector<KDL::Frame>(); // empty pose
}


void stopMoving(){
  // send and empty torque message
  sensor_msgs::JointState msg;

  msg.header.stamp = ros::Time::now();

  msg.position.resize(16);
  msg.velocity.resize(16);
  msg.effort.resize(16);

  for (int j=0; j < 16; j++){
    msg.name.push_back("joint_"+to_string(j+1));
  }

  torque_pub_.publish(msg);
}


/**
* If maintain_pose is not checked, then the control finishes
* when the torque drops below a threshold
*/
bool isFinished(const JntArray& torque_vec){

  // check if stop torque is exceeded
  int t_size = torque_vec.rows();

  // FIXME: one joint is enough for stop
  for(int ji=0; ji<t_size; ji++)
    // compare to threshold
    if(abs(torque_vec(ji))>stop_torque_)
      return false;

  // time out fail check
  ros::Time t_now = ros::Time::now();
  double time_passed = (t_now - t_begin_).sec + 1e-9 * (t_now - t_begin_).nsec;
  // check timeout first
  if(time_passed > time_limit_){

    ROS_WARN("Cartesian Pose Server: timeout");
    stopMoving();

    success_ = false; // FIXME: unused
    return true;
  }

  return true;
}

void publishTorque(const KDL::JntArray& torque_vec){
  // create a joint state message and publish torques
  sensor_msgs::JointState msg;

  int dof = FINGER_COUNT*FINGER_LENGTH;

  msg.header.stamp = ros::Time::now();

  msg.position.resize(dof);
  msg.velocity.resize(dof);

  for (int j=0; j < dof; j++){

    // joint torque is sum of the user input and compensation torques.
    double joint_torque = torque_vec(j);

    // emergency stop
    if(joint_torque > safety_torque_*1.5 || joint_torque < -safety_torque_*1.5){
      ROS_WARN("Too much torque! %.3f", joint_torque);
      stopMoving();
      finished_ = true;
      ros::spinOnce();
      return;
    }

    // shave torque instead of emergency stop
    joint_torque = max(joint_torque, -safety_torque_);
    joint_torque = min(joint_torque, safety_torque_);

    // add torques to message as effort
    msg.effort.push_back(joint_torque);
    // names of joints
    msg.name.push_back("joint_"+to_string(j+1));
  }

  torque_pub_.publish(msg);
}

void limitVector(KDL::Vector& vec, double lim){
  double v_norm = vec.Norm();
  if(v_norm > lim)
    // resize to fit to the limit
    vec = vec / v_norm * lim;
}

// creates the desired velocities for each finger.
// Speed is limited v_lim_vel_ & v_lim_rot_
HandVelocity limitedDesiredVelocity(const HandPose& x_cur){

  HandVelocity xd_des(FINGER_COUNT);

  // we evaluate each finger velocity separately
  for(int fi=0; fi<FINGER_COUNT; fi++){
    // velocity is proportional to distance
    xd_des[fi] = KDL::diff(x_cur[fi], x_des_[fi]);
    // imposing speed as a limit will support slowing near the end
    // applying the limit on the norm
    // Assert |xd_des| < v_lim
    limitVector(xd_des[fi].vel, v_lim_vel_);
    limitVector(xd_des[fi].rot, v_lim_rot_);
  }

  return xd_des;
}


void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg) {

  q_current_ = msg->position;
}

void timerCallback(const ros::TimerEvent&){
  // avoid working after finishing
  if(finished_) return;

  // update controller state and get the current cartesian pose
  if(q_current_.size() == 0) return;
  HandPose x_cur = cp_control_->updatePose(q_current_);

  // *** initial state
  if(!state_read_){

    // if no desired state, then maintain the current state
    if(x_des_.size() != FINGER_COUNT)
      x_des_ = x_cur;

    t_update_ = ros::Time::now();
    state_read_ = true;
    return;
  }

  // *** control

  // Calc time since last control
  ros::Time t_now = ros::Time::now();

  t_update_ = ros::Time::now();

  // calculate the desired velocity, limited by predefined values
  HandVelocity xd_des = limitedDesiredVelocity(x_des_);

  // calculate torques to reach the desired state
  JntArray torques;
  torques = cp_control_->computeTorques(x_des_, xd_des);

  publishTorque(torques);

  // if not maintaining, then stop when finished
  if(!maintain_ && !finished_){
    finished_ = isFinished(torques);

    if(finished_){
      success_ = true;
      stopMoving();
    }
  }

}

bool processPoseRequest(
      allegro_hand_kdl::PoseRequest::Request& req,
      allegro_hand_kdl::PoseRequest::Response& res){

  // read the pose (if exists)
  vector<KDL::Frame> pose = getDefinedPose(req.pose);
  if(pose.size() < FINGER_COUNT) return false;

  // update the desired joint states
  x_des_ = pose;

  // activate control again
  finished_ = false;

  // used when not maintaining the pose:
  state_read_ = false;
  t_begin_ = ros::Time::now();

  // FIXME: success is meaningless right now
  res.success = true;
  return true;
}

// This procedure is called when the ros node is interrupted
void sigintCallback(int sig){

  stopMoving();

  // set finished state
  finished_ = true;

  // spin once to hand communication
  ros::spinOnce();

  // All the default sigint handler does is to call shutdown()
  ros::shutdown();
}
