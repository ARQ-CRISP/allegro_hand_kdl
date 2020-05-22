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

#include "allegro_hand_kdl/cartesian_position_control.h"

using namespace std;
using namespace allegro_hand_kdl;

/*********************************************************************
* CartesianPositionController of allegro_hand_kdl package calculates the
* required torques for converging to desired fingertip cartesian positions
* and velocities, using PID control and manipulator Jacobians.
*********************************************************************/
CartesianPositionController::CartesianPositionController(){

    initParams_();
}
CartesianPositionController::CartesianPositionController(AllegroKdlConfig& kdl_config){

  createKdlObjects(kdl_config);
  initParams_();
}
CartesianPositionController::~CartesianPositionController(){

}

/*********************************************************************
* Internal function, called by every constructor
*********************************************************************/
void CartesianPositionController::initParams_(){
  setGains(10.0, 0.0, 1.0, 0.0);

  // integral control
  decay_int_ = 0.0000;

  // empty state data
  e_sum_vec_.resize(FINGER_COUNT);
  vel_past_.resize(FINGER_COUNT);

  // all fingers are active by default
  active_fingers_.resize(4, true);

}

void CartesianPositionController::createKdlObjects(AllegroKdlConfig& kdl_config){
  cf_solver_.reset(new CartesianForce(kdl_config));
  kin_solver_.reset(new Kinematics(kdl_config));
}
/*********************************************************************
* Given the current cartesian poses (x), calculates the cartesian forces
* needed to achieve desired poses (x_des) and velocities (xd_des).).
* Attention: updates the inner timer //TODO: remove warning when updated
*********************************************************************/
HandAcceleration CartesianPositionController::computeForces(const HandPose& x_des, const HandVelocity& xd_des){
  // memory safety
  if(!checkKdlObjects_()) return HandAcceleration();

  // Calc time since last control
  ros::Time t_now = ros::Time::now();
  double dt_update = (t_now - t_update_).sec + 1e-9 * (t_now - t_update_).nsec;
  // time init? too long delay? the memory variables are outdated
  if(dt_update > 0.5) {
    ROS_INFO("Cartesian position control: Re-initiated control state.");
    // reset past
    dt_update = 0.0;
    x_last_.resize(0);
    resetIntegralError();
  }
  // estimate the current velocity
  HandVelocity xd;
  estimateVelocity_(dt_update, x_cur_, xd);

  // compute desired acceleration using position and velocity
  HandAcceleration xdd_des(FINGER_COUNT);
  for(int fi=0; fi < FINGER_COUNT; fi++){
    if(!active_fingers_[fi]) continue; // skip inactive fingers

    // current position error
    KDL::Twist x_dif = KDL::diff(x_cur_[fi], x_des[fi]);

    // calculate the desired cartesian acceleration
    KDL::Twist xdd_des_twist =
      k_p_ * x_dif + k_d_ * (xd_des[fi] - xd[fi]) + k_i_ * e_sum_vec_[fi];
    // convert to wrench
    // rotation gain for both proportional and derivative parts
    xdd_des[fi].force = xdd_des_twist.vel;
    xdd_des[fi].torque = xdd_des_twist.rot * k_r_;
  }

  // accumulate intergal control error for future
  updateIntegralError_(x_cur_, x_des, dt_update);

  // update time for future
  t_update_ = ros::Time::now(); // TODO: move dt_update to updatePose?

  return xdd_des;
}
/*********************************************************************
* Given the current cartesian poses (x), calculates the torques
* needed to achieve desired poses (x_des) and velocities (xd_des).
* Attention: updates the inner timer
*********************************************************************/
KDL::JntArray CartesianPositionController::computeTorques(const HandPose& x_des, const HandVelocity& xd_des){
  // memory safety
  if(!checkKdlObjects_()) return KDL::JntArray();

  HandAcceleration xdd_des = computeForces(x_des, xd_des);

  // convert to required format by cf_solver
  vector<KDL::JntArray> u_des(FINGER_COUNT);
  for(int fi=0; fi < FINGER_COUNT; fi++){

    // convert to JntArray
    u_des[fi].resize(6);

    if(!active_fingers_[fi]) continue; // skip inactive fingers

    for(int i = 0; i < 6; ++i)
      u_des[fi].data[i] = xdd_des[fi][i];
  }

  // calculate torques to reach the desired state
  return cf_solver_->computeTorques(q_cur_, u_des);
}
// wrapper with std vectors
vector<double> CartesianPositionController::computeTorques(const vector<geometry_msgs::Pose>& x_des, const vector<geometry_msgs::Twist>& xd_des){
  // create kdl data types
    // inputs
  HandPose pose_des(FINGER_COUNT);
  HandVelocity vel_des(FINGER_COUNT);
    // output
  JntArray torq_kdl;

  // convert for each finger
  for(int fi=0; fi<FINGER_COUNT; fi++){
    tf::poseMsgToKDL(x_des[fi], pose_des[fi]);
    tf::twistMsgToKDL(xd_des[fi], vel_des[fi]);
  }

  // call the core method
  torq_kdl = computeTorques(pose_des, vel_des);

  // convert output to std
  vector<double> torq_out;
  kdl_control_tools::vectorKdlToStd(torq_kdl, torq_out);
  return torq_out;
}

/*********************************************************************
* Updates the current pose using the current joint state q and
* its internal forward kinematics solver. Returns the resulting hand pose.
*********************************************************************/
HandPose CartesianPositionController::updatePose(const JntArray& q){
  // memory safety
  if(!checkKdlObjects_()) return x_cur_;

  // update q_cur_
  q_cur_ = q;
  // update x_cur_
  kin_solver_->calcCartPos(q_cur_, x_cur_);

  return x_cur_;
}
// std vector interface
HandPose CartesianPositionController::updatePose(const vector<double>& q){
  // memory safety
  if(!checkKdlObjects_()) return x_cur_;

  // update q_cur_
  kdl_control_tools::vectorStdToKdl(q, q_cur_);
  // update x_cur_
  kin_solver_->calcCartPos(q_cur_, x_cur_);

  return x_cur_;
}

void CartesianPositionController::updatePose(const HandPose& x){
  // update x_cur_
  x_cur_ = x;
}
/*********************************************************************
* Check if necessary KDL objects are created
*********************************************************************/
bool CartesianPositionController::checkKdlObjects_() {
  // memory safety
  if(!cf_solver_ || !kin_solver_) {
    ROS_WARN("Cartesian position control: KDL objects are not created!");
    return false;
  }

  return true;
}
/*********************************************************************
* adding of single dimension of error
* if the sign changed, the error resets
*********************************************************************/
double CartesianPositionController::addIntegralError_(const double e_total, const double e_last) {
  // different signs?
  if(e_total * e_last < 0)
    // reset the error
    return e_last;
  else
    // accumulate error
    return (e_total * (1.0-decay_int_)) + e_last;
}

/*********************************************************************
* Accumulate the integral error for future. Apply decay to past error.
*********************************************************************/
void CartesianPositionController::updateIntegralError_(const HandPose& x, const HandPose& x_des, double t) {
  // add the last error to integral error
  for(int fi=0; fi<FINGER_COUNT; fi++){
    if(!active_fingers_[fi]) continue; // skip inactive fingers

    KDL::Twist err_last =  KDL::diff(x[fi], x_des[fi]) * t;

    for(int di=0; di<3; di++) { // dimensions
      e_sum_vec_[fi][di] = addIntegralError_(e_sum_vec_[fi][di], err_last.vel[di]);
      e_sum_vec_[fi][di+3] = addIntegralError_(e_sum_vec_[fi][di+3], err_last.rot[di]);
    }
  }
}
/*********************************************************************
* Use internal timer to estimate velocity since last control
*********************************************************************/
void CartesianPositionController::estimateVelocity_(const double dt_update, const HandPose& x, HandVelocity& xd){

    // if x_last_ doesn't exist
    if(x_last_.size() < FINGER_COUNT){
      // zero velocity array
      xd.resize(FINGER_COUNT);

    // if no time passed yet
    }else if(dt_update <= 0.0){
      ROS_WARN("Cartesian position control: delta time is 0, multiple calls at the same moment!");
      xd = vel_past_;
      return;

    // otherwise, the normal case
    }else{
      // differentiate x
      for(int fi=0; fi<FINGER_COUNT; fi++){
        KDL::Twist xd_cur = KDL::diff(x_last_[fi], x[fi], dt_update);
        // smooth
        xd_cur = (vel_past_[fi] *0.4 + xd_cur *0.6);
        vel_past_[fi] = xd_cur;
      }

      xd = vel_past_;
    }
    // update state
    x_last_ = x;
}
/*********************************************************************
* Reset the accumulated integral error. Useful when changing goals.
*********************************************************************/
void CartesianPositionController::resetIntegralError(){

  e_sum_vec_.clear();
  e_sum_vec_.resize(FINGER_COUNT);
}

/*********************************************************************
* Setters & getters
*********************************************************************/
// setters
void CartesianPositionController::setGains(const double k_pos, const double k_rot, const double k_vel, const double k_int)
{ setPositionGain(k_pos);
  setRotationGain(k_rot);
  setVelocityGain(k_vel);
  setIntegralGain(k_int); }
void CartesianPositionController::setPositionGain(const double k_pos)
{ k_p_ = k_pos; }
void CartesianPositionController::setRotationGain(const double k_rot)
{ k_r_ = k_rot; }
void CartesianPositionController::setVelocityGain(const double k_vel)
{ k_d_ = k_vel; }
void CartesianPositionController::setIntegralGain(const double k_int)
{ k_i_ = k_int; }
void CartesianPositionController::setIntegralDecay(const double decay_rate)
{ decay_int_ = decay_rate; }
void CartesianPositionController::setActiveFingers(const int active_digits)
{
  // check digits by modulus
  for(int fi=0; fi<FINGER_COUNT; fi++){
    int digit_no = FINGER_COUNT-fi-1;
    int digit_val = pow(10, digit_no);
    active_fingers_[fi] = (active_digits)%(digit_val*10) >= digit_val;
  }
}
void CartesianPositionController::setActiveFingers(const vector<bool> activity_vec)
{ active_fingers_ = activity_vec; }
void CartesianPositionController::setActiveFingers(const vector<uint8_t> activity_vec)
{
  for(int fi=0; fi<FINGER_COUNT; fi++)
    active_fingers_[fi] = (bool) activity_vec[fi];
}
// getters
double CartesianPositionController::getPositionGain()
{ return k_p_; }
double CartesianPositionController::getRotationGain()
{ return k_r_; }
double CartesianPositionController::getVelocityGain()
{ return k_d_; }
double CartesianPositionController::getIntegralGain()
{ return k_i_; }
double CartesianPositionController::getIntegralDecay()
{ return decay_int_; }
