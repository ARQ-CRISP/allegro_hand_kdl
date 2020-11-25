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
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float64MultiArray.h>

using namespace std;
using namespace allegro_hand_kdl;

// A HandPose is a vector<KDL::Frame>
typedef shared_ptr<HandPose> HandPosePtr;

unique_ptr<CartesianPositionController> cp_control_;

ros::Publisher torque_pub_; // to control the robot
ros::Publisher vis_pub_; // to visualize the desired frames
ros::Publisher err_pub_; // to publish the pose error

// desired average speed
double v_lim_vel_ = 0.03; // meters per second
double v_lim_rot_ = 0.35; // radians per second

// params
double update_freq_ = 200;
double safety_torque_ = 1.0;
bool maintain_ = true;
double stop_err_ = 0.0005; // meters
double time_limit_ = 5.0;
string pose_param_ns_;

double k_p_ = 10;
double k_r_ = 1;
double k_d_ = 0.001;
double k_i_ = 0.0;

// state memory
vector<double> q_current_;
HandPose x_des_;

vector<double> total_errors_;

ros::Time t_update_;
ros::Time t_begin_;

bool success_ = false;

bool finished_ = true;

void sigintCallback(int sig);
vector<KDL::Twist> publishError(const HandPose& x_cur);
void publishTorque(const KDL::JntArray& torque_vec);
void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg);
void timerCallback(const ros::TimerEvent&);
bool processPoseRequest(allegro_hand_kdl::PoseRequest::Request& req, allegro_hand_kdl::PoseRequest::Response& res);
void displayDesiredPoses(const HandPose& x_des);
HandVelocity limitedDesiredVelocity(const HandPose& x_cur);
HandPose getDefinedPose(string desired_name);
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
  // NOTE: debug
  vis_pub_ =
    nh.advertise<visualization_msgs::MarkerArray>("desired_pose_markers", 1);
  err_pub_ =
    nh.advertise<std_msgs::Float64MultiArray>("cartesian_pose_error", 1);

  ros::ServiceServer service =
    nh.advertiseService("desired_cartesian_pose", processPoseRequest);

  // start the control loop
  t_begin_ = ros::Time::now();
  ros::Timer timer = nh.createTimer(ros::Duration(1.0/update_freq_), &timerCallback);

  // NOTE: debug
  if(x_des_.size() == FINGER_COUNT)
    displayDesiredPoses(x_des_);
  // monitor errors [e_pos, e_rot] x fingers
  total_errors_.resize(2*FINGER_COUNT, 0.0);

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
    HandPose pose = getDefinedPose(str_pose);
    if(pose.size() == FINGER_COUNT){
      // assign the pose and activate controller
      x_des_ = pose;
      finished_ = false;
      // inform
      ROS_INFO("Cartesian Pose Server: Initial pose is %s.", str_pose.c_str());
    }
  }

  // optional frequency parameter
  if(ros::param::has("~hz"))
    ros::param::get("~hz", update_freq_);

  // optional velocity limits (meters, radians)
  if(ros::param::has("~velocity_limit_linear"))
    ros::param::get("~velocity_limit_linear", v_lim_vel_);

  if(ros::param::has("~velocity_limit_angular"))
    ros::param::get("~velocity_limit_angular", v_lim_rot_);

  return true;
}

// get pose from server whenever necessary
HandPose getDefinedPose(string desired_name){

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

  ROS_WARN("Cartesian Pose Server: can't find the pose %s", desired_name.c_str());
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
bool isFinished(const vector<KDL::Twist>& err_vec){

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

  // stop when all errors are lower than the threshold
  for(int fi=0; fi<FINGER_COUNT; fi++)
    // compare to threshold
    if(err_vec[fi].vel.Norm()>stop_err_)
      return false;

  return true;
}

vector<KDL::Twist> publishError(const HandPose& x_cur){

  std_msgs::Float64MultiArray msg_err;
  vector<KDL::Twist> instant_errors(FINGER_COUNT);

  for(int fi=0; fi<FINGER_COUNT; fi++){
    // calculate instant and total errors
    instant_errors[fi] = KDL::diff(x_cur[fi], x_des_[fi]);
    total_errors_[fi*2] += instant_errors[fi].vel.Norm();
    total_errors_[fi*2+1] += instant_errors[fi].rot.Norm();
    // position
    for(int di=0; di<3; di++) // add dimensions
      msg_err.data.push_back(instant_errors[fi].vel.data[di]);
    // orientation
    for(int di=0; di<3; di++) // add dimensions
      msg_err.data.push_back(instant_errors[fi].rot.data[di]);
  }

  err_pub_.publish(msg_err);

  return instant_errors;

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

      // warn user of too much torque
    if(joint_torque > safety_torque_ || joint_torque < -safety_torque_)
    {
      ROS_WARN("Too much torque! %.3f, joint %d", joint_torque, j);
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
HandVelocity limitedDesiredVelocity(const HandPose& x_cur)
{
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


void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  q_current_ = msg->position;
}


void timerCallback(const ros::TimerEvent&)
{
  // avoid working after finishing
  if(finished_ || q_current_.size() == 0) return;

  // update controller state and get the current cartesian pose
  HandPose x_cur = cp_control_->updatePose(q_current_);

  // if no desired state, then maintain the current state
  if(x_des_.size() != FINGER_COUNT)
    x_des_ = x_cur;
  // calculate the desired velocity, limited by predefined values
  HandVelocity xd_des = limitedDesiredVelocity(x_des_);

  // calculate torques to reach the desired state
  JntArray torques;
  torques = cp_control_->computeTorques(x_des_, xd_des);

  publishTorque(torques);
  vector<KDL::Twist> err = publishError(x_cur);

  // if not maintaining, then stop when finished
  if(!maintain_ && !finished_){
    finished_ = isFinished(err);

    if(finished_){
      success_ = true;
      stopMoving();

      // NOTE: debug
      ros::Time t_now = ros::Time::now();
      double time_passed = (t_now - t_begin_).sec + 1e-9 * (t_now - t_begin_).nsec;

      ROS_INFO("Finished moving to the pose in %.2fs. Errors:", time_passed);
      cout << "[ ";
      for(int fi=0; fi<FINGER_COUNT; fi++)
        cout << "(" << total_errors_[fi*2]/time_passed << ", " << total_errors_[fi*2+1]/time_passed << ") ";
      cout << "]" << endl << endl;

    }
  }

}

bool processPoseRequest(
      allegro_hand_kdl::PoseRequest::Request& req,
      allegro_hand_kdl::PoseRequest::Response& res)
{
  // read the pose (if exists)
  if(req.pose.size() > 0){
    HandPose pose = getDefinedPose(req.pose);
    if(pose.size() < FINGER_COUNT) return false;
    // update the desired joint states
    x_des_ = pose;
    displayDesiredPoses(pose);
  }else{
    // otherwise maintain current pose
    x_des_.clear();
  }
  // display on RViz

  // maintaining behaviour (optional)
  if(req.behaviour.size() > 0)
    maintain_ = (req.behaviour == "maintain");
  // finger activation update (optional)
  if(req.active_fingers.size() == FINGER_COUNT)
    cp_control_->setActiveFingers(req.active_fingers);

  // activate control again
  finished_ = false;

  // used when not maintaining the pose:
  t_begin_ = ros::Time::now();

  // monitor errors [e_pos, e_rot] x fingers
  total_errors_.clear();
  total_errors_.resize(2*FINGER_COUNT, 0.0);

  // FIXME: success is meaningless right now
  res.success = true;
  return true;
}

// This procedure is called when the ros node is interrupted
void sigintCallback(int sig)
{
  stopMoving();

  // set finished state
  finished_ = true;

  // spin once to hand communication
  ros::spinOnce();

  // All the default sigint handler does is to call shutdown()
  ros::shutdown();
}

void displayDesiredPoses(const HandPose& x_des)
{
  visualization_msgs::MarkerArray marker_arr;

  for(int fi=0; fi<FINGER_COUNT; fi++){

    visualization_msgs::Marker marker;
    marker.header.frame_id = "hand_root";
    marker.header.stamp = ros::Time();
    marker.ns = "fingertip_pose_targets";

    marker.id = fi;

    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;

    marker.scale.x = 0.015;
    marker.scale.y = 0.003;
    marker.scale.z = 0.003;
    marker.color.r = 0.9;
    marker.color.g = 0.6;
    marker.color.b = 0.5;
    marker.color.a = 0.6;

    // arrow pos and rot
    tf::poseKDLToMsg(x_des[fi], marker.pose);

    marker.lifetime = ros::Duration(time_limit_*2);
    marker.frame_locked = true;

    marker_arr.markers.push_back(marker);
  }

  vis_pub_.publish( marker_arr );
}
