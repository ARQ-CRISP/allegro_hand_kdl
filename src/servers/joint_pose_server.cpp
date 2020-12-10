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
* This is a server node for JointPositionController.
* The goal of the control is to maintain the initial positions
* of finger joints with zero velocity.
* If a PoseRequest service is issued, then the fingers are moved
* to the predefined pose states.
* Control gains are read from parameters.
**/

#include <signal.h>

#include <allegro_hand_kdl/joint_position_control.h>
#include <allegro_hand_kdl/PoseRequest.h>

#include <kdl_control_tools/progress_logger.h>

#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

using namespace std;
using namespace allegro_hand_kdl;

JointPositionController* joint_control_;

ros::Publisher torque_pub_;  // to control the robot
ros::Publisher error_pub_;  // for debugging
ros::Publisher dt_pub_;  // for debugging

// desired average joint speed (radians per second)
double v_lim_ = 0.3;

// params
double update_freq_ = 200;
double safety_torque_ = 1.0;
bool maintain_ = true;
double stop_torque_ = 0.05;
double time_limit_ = 3.0;
string pose_param_ns_;

// state memory
vector<double> q_current_;
vector<double> q_des_;

ros::Time t_begin_;
ros::Time t_update_;

bool success_ = false;

bool finished_ = true;

void sigintCallback(int sig);
void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg);
void timerCallback(const ros::TimerEvent&);
bool processPoseRequest(allegro_hand_kdl::PoseRequest::Request& req, allegro_hand_kdl::PoseRequest::Response& res);
vector<double> getDefinedPose(string desired_name);
bool getControlGains(ros::NodeHandle& nh, vector<double>& k_pos_vec, vector<double>& k_vel_vec, vector<double>& k_int_vec);
bool getParams();

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joint_pose_server");
  ROS_INFO("Joint Pose server node");

  ros::NodeHandle nh;

  // attempt to get ros parameters
  if (!getParams()) return -1;

  // ros shutdown handler
  signal(SIGINT, sigintCallback);

  joint_control_ = new JointPositionController();

  // get the pd gain params if exists
  vector<double> k_pos_vec;
  vector<double> k_vel_vec;
  vector<double> k_int_vec;

  if (getControlGains(nh, k_pos_vec, k_vel_vec, k_int_vec))
  {
    joint_control_->setGains(k_pos_vec, k_vel_vec, k_int_vec);
  }
  else
  {
    joint_control_->setGains(0.4, 0.05, 0.0);
  }

  // integral decay
  joint_control_->setIntegralDecay(0.001);

  // determine active fingers
  joint_control_->setActiveFingers(91111);

  // countdown only if there is an initial pose
  if (q_des_.size() > 0)
  {
    finished_ = false;

    ros::Duration sleep_time(1.0);

    for (int ci = 3; ci > 0; ci--)
    {
      ROS_INFO("Joint Pose Server: starting in %d", ci);
      sleep_time.sleep();
    }
  }

  ROS_INFO("Joint Pose Server: started!");

  // create ros communication nodes
  torque_pub_ =
    nh.advertise<sensor_msgs::JointState>("position_torque", 1);
  ros::Subscriber sub_js =
    nh.subscribe<sensor_msgs::JointState>("joint_states", 1, jointStateCallback, ros::TransportHints().tcpNoDelay().reliable());

  ros::ServiceServer service =
    nh.advertiseService("desired_pose", processPoseRequest);

  // debugging publisher
  error_pub_ =
    nh.advertise<std_msgs::Float64MultiArray>("joint_control_errors", 1);
  dt_pub_ =
    nh.advertise<std_msgs::Float64>("joint_control_dt", 1);

  // start the control loop
  t_begin_ = ros::Time::now();
  ros::Timer timer = nh.createTimer(ros::Duration(1.0/update_freq_), &timerCallback);

  ros::spin();

  delete joint_control_;

  return 0;

}


bool getControlGains(ros::NodeHandle& nh, vector<double>& k_pos_vec, vector<double>& k_vel_vec, vector<double>& k_int_vec)
{
  k_pos_vec.resize(FINGER_LENGTH*FINGER_COUNT);
  k_vel_vec.resize(FINGER_LENGTH*FINGER_COUNT);
  k_int_vec.resize(FINGER_LENGTH*FINGER_COUNT);

  string param_name;
  if (!nh.searchParam("/allegroHand_right_0/gains/pose", param_name) )
  {
    ROS_ERROR("Joint Pose Server: Can't find gains param.");
    return false;
  }

  for (int fi=0; fi < FINGER_COUNT; fi++)
    for (int si=0; si < FINGER_LENGTH; si++)
    {
      ros::param::get(param_name+"/p/j"+to_string(fi)+to_string(si),
        k_pos_vec[fi*FINGER_LENGTH+si]);
      ros::param::get(param_name+"/d/j"+to_string(fi)+to_string(si),
        k_vel_vec[fi*FINGER_LENGTH+si]);
      ros::param::get(param_name+"/i/j"+to_string(fi)+to_string(si),
        k_int_vec[fi*FINGER_LENGTH+si]);
    }

  return true;
}

// get external parameters if possible
bool getParams()
{
  // time limit
  if(!ros::param::get("~time_limit", time_limit_))
  {
    ROS_WARN("Joint Pose Server: Can't get time limit param.");
  }

  // safety_torque
  if (!ros::param::get("/allegro_kdl/safety_torque", safety_torque_))
  {
    ROS_WARN("Joint Pose Server: Can't get safety_torque param. Assuming %.3f.", safety_torque_);
  }
  ROS_DEBUG("Joint Pose Server: safety_torque is %.3f.", safety_torque_);

  // get the param namespace param if specified
  if (!ros::param::get("~pose_ns", pose_param_ns_))
  {
    pose_param_ns_ = "allegro_kdl/poses";
    ROS_INFO("Joint Pose Server: Assuming default namespace %s.", pose_param_ns_.c_str());
  }
  pose_param_ns_ += "/joint_poses";
  // make sure that the params exist
  if (!ros::param::has(pose_param_ns_))
  {
    ROS_ERROR("Joint Pose Server: Can't find poses %s", pose_param_ns_.c_str());
    return false;
  }

  // maintain pose
  if (!ros::param::get("~maintain_pose", maintain_))
  {
    ROS_WARN("Joint Pose Server: Can't get maintain_pose param.");
  }
  if (maintain_)
  {
    finished_ = false;
    ROS_INFO("Joint Pose Server: maintaining pose.");
  }

  // maintain pose
  string str_pose;
  if (ros::param::get("~initial_pose", str_pose))
  {
    // get the pose if exists
    vector<double> pose = getDefinedPose(str_pose);
    if (pose.size() == FINGER_COUNT*FINGER_LENGTH)
    {
      // assign the pose and activate controller
      q_des_ = pose;
      finished_ = false;
      // inform
      ROS_INFO("Joint Pose Server: Initial pose is %s.", str_pose.c_str());
    }
  }

  // optional frequency parameter
  if (ros::param::has("~hz"))
    ros::param::get("~hz", update_freq_);

  // optional velocity limit parameter (radians, same for all joints)
  if (ros::param::has("~velocity_limit"))
    ros::param::get("~velocity_limit", v_lim_);

  return true;

}

// returns the defined pose from ROS param server
// returns an empty vector if it doesn't exist
vector<double> getDefinedPose(string desired_name)
{
  // iterate existing poses (following pattern p0,p1...) until finding the one
  int pi = 0;
  while (ros::param::has(pose_param_ns_+"/p"+to_string(pi)))
  {
    string name;
    ros::param::get(pose_param_ns_+"/p"+to_string(pi)+"/name", name);

    // skip until match
    if (name != desired_name)
    {
      pi++;
      continue;
    }

    vector<double> jstate;
    ros::param::get(pose_param_ns_+"/p"+to_string(pi)+"/state", jstate);

    return jstate;
  }

  return vector<double>();  // empty means failed
}

void stopMoving()
{
  // send and empty torque message
  sensor_msgs::JointState msg;

  msg.header.stamp = ros::Time::now();

  msg.position.resize(16);
  msg.velocity.resize(16);
  msg.effort.resize(16);

  for (int j=0; j < 16; j++)
  {
    msg.name.push_back("joint_"+to_string(j));
  }

  torque_pub_.publish(msg);
}

/**
* If maintain_pose is not checked, then the control finishes
* when the torque drops below a threshold
*/
bool isFinished(const vector<double>& torque_vec)
{
  // check if stop torque is exceeded
  int t_size = torque_vec.size();

  // time out fail check
  ros::Time t_now = ros::Time::now();
  double time_passed = (t_now - t_begin_).sec + 1e-9 * (t_now - t_begin_).nsec;
  if (time_passed > time_limit_)
  {
    ROS_WARN("Joint Pose Server: timeout");
    stopMoving();
    finished_ = true;

    success_ = false;  // FIXME: unused
    return true;
  }

  // stop if all torques are lower than the threshold
  for (int ji=0; ji < t_size; ji++)
    // compare to threshold
    if (abs(torque_vec[ji]) > stop_torque_)
      return false;

  return true;
}

void publishError(const vector<double>& torque_vec)
{
    std_msgs::Float64MultiArray msg;
    msg.data = torque_vec;
    error_pub_.publish(msg);
}

void publishTorque(const vector<double>& torque_vec)
{
  // create a joint state message and publish torques
  sensor_msgs::JointState msg;

  int dof = FINGER_COUNT*FINGER_LENGTH;

  msg.header.stamp = ros::Time::now();

  msg.position.resize(dof);
  msg.velocity.resize(dof);

  for (int j=0; j < dof; j++)
  {
    // joint torque is sum of the user input and compensation torques.
    double joint_torque = torque_vec[j];


    // warn user of too much torque
    if(joint_torque > safety_torque_ || joint_torque < -safety_torque_)
    {
      ROS_WARN("Joint pose server: Too much torque! %.3f, j: %d", joint_torque, j);
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

void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  q_current_ = msg->position;
}

void timerCallback(const ros::TimerEvent&)
{
  // avoid working after finishing
  if (finished_ || q_current_.size() == 0) return;

  // Calc time since last control
  ros::Time t_now = ros::Time::now();
  double dt_update = (t_now - t_update_).sec + 1e-9 * (t_now - t_update_).nsec;
  t_update_ = ros::Time::now();
  // publish dt TODO: organize debug
  std_msgs::Float64 dt_msg;
  dt_msg.data = dt_update;
  dt_pub_.publish(dt_msg);

  // if no desired state, then maintain the current state
  if (q_des_.size() < FINGER_LENGTH*FINGER_COUNT)
    q_des_ = q_current_;

  // qd_des is the desired average velocity
  vector<double> qd_des(FINGER_COUNT*FINGER_LENGTH);
  for (int qi=0; qi < FINGER_COUNT*FINGER_LENGTH; qi++)
  {
    // velocity is proportional to distance / per second
    qd_des[qi] = q_des_[qi] - q_current_[qi];
    // imposing speed as a limit will support slowing near the end
    // Assert -v_lim < qd_des < v_lim
    qd_des[qi] = min(qd_des[qi], v_lim_);
    qd_des[qi] = max(qd_des[qi], -v_lim_);
  }

  // calculate torques to reach the desired state
  vector<double> torques;
  joint_control_->computeTorques(q_current_, q_des_, qd_des, torques);

  publishTorque(torques);

  // if not maintaining, then stop when finished
  if (!maintain_ && !finished_)
  {
    finished_ = isFinished(torques);

    if (finished_)
    {
      success_ = true;
      stopMoving();
    }
  }
}

bool processPoseRequest(
      allegro_hand_kdl::PoseRequest::Request& req,
      allegro_hand_kdl::PoseRequest::Response& res)
  {
  // read the pose, if exists
  if (req.pose.size() > 0)
  {
    vector<double> pose = getDefinedPose(req.pose);
    if (pose.size() < FINGER_COUNT*FINGER_LENGTH) return false;
    // update the desired joint states
    q_des_ = pose;
  }
  else
  {
    // otherwise maintain current pose
    q_des_.clear();
  }

  // maintaining behaviour (optional)
  if (req.behaviour.size() > 0)
    maintain_ = (req.behaviour == "maintain");
  // finger activation update (optional)
  if (req.active_fingers.size() == FINGER_COUNT)
    joint_control_->setActiveFingers(req.active_fingers);

  // activate controller again
  finished_ = false;

  // used when not maintaining the pose:
  t_begin_ = ros::Time::now();

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
