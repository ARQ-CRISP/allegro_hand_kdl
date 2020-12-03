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
#include <signal.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>

#include <kdl/kdl.hpp>
#include <kdl_conversions/kdl_msg.h>

#include <allegro_hand_kdl/cartesian_force.h>

using namespace std;
using namespace allegro_hand_kdl;

// kdl interface objects
AllegroKdlConfig allegro_kdl_config;
CartesianForce* cf_solver;

// control parameters
double envelop_scale_ = 1.2; // get from rosparam
double safety_torque_ = 1.0; // get from rosparam

// ros communication objects
ros::Publisher torque_publisher; // to control the robot
vector<ros::Publisher> wrench_publisher_vec; // to visualize in rviz
tf2_ros::Buffer tfBuffer;
// ros communication data
vector<double> joint_pos_vec;

void createForceVec(const geometry_msgs::Transform& from, const geometry_msgs::Transform & to, vector<double> & force_vec);
void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg);
void intensityCallback(const std_msgs::Float64::ConstPtr &msg);
void timerCallback(const ros::TimerEvent&);
void sigintCallback(int sig);
bool getParams(ros::NodeHandle nh);

int main(int argc, char** argv){
  ros::init(argc, argv, "allegro_hand_envelop_force");

  ros::NodeHandle nh;

  // attempt to get ros parameters
  if(!getParams(nh)) return -1;

  // create cartesian force solver
  allegro_kdl_config.parseKdl(nh);
  cf_solver = new CartesianForce(allegro_kdl_config);

  // create torque publisher
  torque_publisher =
    nh.advertise<sensor_msgs::JointState>("envelop_torque", 3);

  // create wrench publisher to visualize in rviz
  for (int fi=0; fi<FINGER_COUNT; fi++){
    wrench_publisher_vec.push_back(
      nh.advertise<geometry_msgs::WrenchStamped>("finger_wrench_"+to_string(fi), 3));
  }

  // create tf listener
  tf2_ros::TransformListener tfListener(tfBuffer);


  // create float64 listener for intensity
  ros::Subscriber sub_intensity =
    nh.subscribe<std_msgs::Float64>("envelop_intensity", 3, intensityCallback);

  // create joint state listener
  joint_pos_vec.resize(16);
  ros::Subscriber sub_state =
    nh.subscribe<sensor_msgs::JointState>( "joint_states", 3, jointStateCallback);


  ros::Timer timer = nh.createTimer(ros::Duration(0.005), timerCallback);

  // ros shutdown handler
  signal(SIGINT, sigintCallback);

  // wait a while before applying torque
  ros::Duration(1.0).sleep();
  ros::spin();

  delete cf_solver;

  return 0;
}

// get external parameters if possible
bool getParams(ros::NodeHandle nh){
  // envelop_intensity

  ros::param::get("~envelop_scale", envelop_scale_);

  ROS_DEBUG("Envelop force: envelop scale is %f.", envelop_scale_);

  string param_name;
  if(!nh.searchParam("/allegro_kdl/safety_torque", param_name) ){
    ROS_ERROR("Envelop force: Can't find safety_torque param.");
    return false;
  }
  ros::param::get(param_name, safety_torque_);

  ROS_DEBUG("Envelop force: safety torque is %f.", safety_torque_);

  return true;

}

void changeWrenchFrame(const geometry_msgs::Transform& frame_trans, geometry_msgs::Wrench& wrench){
    // NOTE: torques are ignored and zero'ed
    // create kdl data types
    Vector w_kdl;
    tf::vectorMsgToKDL(wrench.force, w_kdl);

    Rotation rot_kdl =
      KDL::Rotation::Quaternion(frame_trans.rotation.x, frame_trans.rotation.y,
                                frame_trans.rotation.z, frame_trans.rotation.w);


    // change reference point of the wrench
    w_kdl = rot_kdl.Inverse() * w_kdl;

    // modify the return vector
    tf::vectorKDLToMsg(w_kdl, wrench.force);
}

void createForceVec(const geometry_msgs::Transform& from, const geometry_msgs::Transform & to, vector<double> & force_vec){

  force_vec.resize(6, 0.0); // 3D force + 3D torque

  force_vec[0] = to.translation.x - from.translation.x;
  force_vec[1] = to.translation.y - from.translation.y;
  force_vec[2] = to.translation.z - from.translation.z;

  // normalize vector and scale
  double magnitude = sqrt(
    force_vec[0]*force_vec[0] +
    force_vec[1]*force_vec[1] +
    force_vec[2]*force_vec[2]);

  force_vec[0] /= magnitude;
  force_vec[1] /= magnitude;
  force_vec[2] /= magnitude;

}

// Wrenches must be unit vectors
void publishFingerWrench(const int finger_no, const vector<double>& wrench_vec, const geometry_msgs::Transform& ff_frame){


  geometry_msgs::WrenchStamped msg;

  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "link_" + to_string(finger_no * FINGER_LENGTH + 3) +"_tip";

  msg.wrench.force.x = wrench_vec[0];
  msg.wrench.force.y = wrench_vec[1];
  msg.wrench.force.z = wrench_vec[2];

  msg.wrench.torque.x = wrench_vec[3];
  msg.wrench.torque.y = wrench_vec[4];
  msg.wrench.torque.z = wrench_vec[5];


  // express the resulting force in fingertip frame
  changeWrenchFrame(ff_frame, msg.wrench);

  wrench_publisher_vec[finger_no].publish(msg);

}


void visualizeFingerWrenches(const vector< vector<double> > wrench_vec, const vector<geometry_msgs::TransformStamped>& ff_frames){

  for (int fi=0; fi<FINGER_COUNT; fi++){
    publishFingerWrench(fi, wrench_vec[fi], ff_frames[fi].transform);
  }
}

void getCenterFrame(const vector<geometry_msgs::TransformStamped>& finger_tf, geometry_msgs::Transform& tf_center){

  double total_weight = FINGER_COUNT + 2.0;

  // weighted average of the positions
  for(int ti=0; ti<FINGER_COUNT; ti++){
    // thumb has more weight
    double weight = 1.0;
    if(ti==3) weight = 3.0;

    tf_center.translation.x += finger_tf[ti].transform.translation.x * weight / total_weight;
    tf_center.translation.y += finger_tf[ti].transform.translation.y * weight / total_weight;
    tf_center.translation.z += finger_tf[ti].transform.translation.z * weight / total_weight;
  }
}

void publishTorques(const vector<double> &tau){
  // create a joint state message and publish torques
  sensor_msgs::JointState msg;

  msg.header.stamp = ros::Time::now();

  int dim = tau.size();

  msg.position.resize(dim);
  msg.velocity.resize(dim);

  for (int j=0; j < dim; j++){
    // scale
    double joint_torque = tau[j];

    // emergency stop
    if(joint_torque > safety_torque_ || joint_torque < -safety_torque_){
      ROS_WARN("Envelop force: Too much torque!");
      sigintCallback(SIGINT);
      return;
    }

    // add torques to message as effort
    msg.effort.push_back(joint_torque);
    // names of joints
    msg.name.push_back("joint_"+to_string(j));
  }

  torque_publisher.publish(msg);
}

void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg) {
  // update the state
  joint_pos_vec = msg->position;
}

// allow modification of intensity (torque_scale) by messages
void intensityCallback(const std_msgs::Float64::ConstPtr &msg) {
  // safety check
  if (envelop_scale_ < 0.0) {
    ROS_WARN("Envelop force: Ignoring non-positive intensity %f", envelop_scale_);
    return;
  }
  // update the intensity
  envelop_scale_ = msg->data;
}

void timerCallback(const ros::TimerEvent&){
  // get the tf transforms of fingers and the thumb
  vector<geometry_msgs::TransformStamped> transformStamped;
  try{
    // transforms of finger tips w.r.t. hand root
    for (int fi=0; fi<FINGER_COUNT; fi++){
      int tip_index = fi * FINGER_LENGTH + 3;
      transformStamped.push_back(
        tfBuffer.lookupTransform(
          "hand_root",
          "link_"+ to_string(tip_index) +"_tip",
          ros::Time(0))
      );
    }
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
    return;
  }

  // calculate force direction from fingers to the thumb
  std::vector< std::vector<double> > force_vec;
  vector<double> tmp_vec;

  geometry_msgs::Transform center_point;
  getCenterFrame(transformStamped, center_point);

  // create a force vector from each fingertip to the center
  for(int fi=0; fi<FINGER_COUNT; fi++){
    createForceVec(transformStamped[fi].transform, center_point, tmp_vec);
    force_vec.push_back(tmp_vec); // concat to force_vec
  }


  for (int fi=0; fi<FINGER_COUNT; fi++)
    for (int wi=0; wi<6; wi++){
      // apply scaling to envelop forces
      force_vec[fi][wi] *= envelop_scale_;
      // scale the thumb force down to balance
      if (fi==3) force_vec[fi][wi] *= 0.8;
    }


  // publish envelop force for visualization
  if(envelop_scale_ > 0.0)
    visualizeFingerWrenches(force_vec, transformStamped);

  // calculate the joint torques needed to achieve envelop forces
  vector<double> torque_joint;
  torque_joint = cf_solver->computeTorques(joint_pos_vec, force_vec);

  publishTorques(torque_joint);
}

void sigintCallback(int sig)
{
  ROS_INFO("Envelop force: cleaning up on exit");
  // send and empty torque message
  sensor_msgs::JointState msg;

  msg.header.stamp = ros::Time::now();

  msg.position.resize(16, 0.0);
  msg.velocity.resize(16, 0.0);
  msg.effort.resize(16, 0.0);

  for (int j=0; j < 16; j++){
    msg.name.push_back("joint_"+to_string(j));
  }

  torque_publisher.publish(msg);
  ros::spinOnce();

  // send empty finger wrench messages and empty transforms
  vector< vector<double> > force_vec(FINGER_COUNT);
  for(int fi; fi<FINGER_COUNT; fi++)
    force_vec[fi].resize(6, 0.0);
  vector<geometry_msgs::TransformStamped> transformStamped;
  visualizeFingerWrenches(force_vec, transformStamped);

  // spin once to hand communication
  ros::spinOnce();

  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}
