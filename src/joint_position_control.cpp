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

#include <allegro_hand_kdl/joint_position_control.h>

using namespace std;
using namespace KDL;

using namespace allegro_hand_kdl;

/*********************************************************************
* JointPositionController of allegro_hand_kdl package calculates the
* required torques for converging to desired joint positions and velocities.
* It uses PID control.
*********************************************************************/
JointPositionController::JointPositionController()
{
  setGains(0.1, 0.01, 0.0);

  // integral control
  decay_int_ = 0.0000;

  // zeros for eigen data
  e_sum_vec_.resize(FINGER_COUNT*FINGER_LENGTH);
  vel_past_.resize(FINGER_COUNT*FINGER_LENGTH);

  for(int i=0; i< FINGER_COUNT*FINGER_LENGTH; i++){
    e_sum_vec_(i) = 0;
    vel_past_(i) = 0;
  }

  // all fingers are active by default
  active_fingers_.resize(4, true);

  // no damping by default
  damp_ratio_ = 0.0;

}
// destructor
JointPositionController::~JointPositionController()
{
  // id_solver__ and p_logger_ are external resources, hence not deleted
}
/*********************************************************************
* Given the current joint positions (q), calculates the torques (torq_out)
* needed to achieve desired joint positions (q_des) and velocities (qd_des).
*********************************************************************/
void JointPositionController::computeTorques(const JntArray& q, const JntArray& q_des, const JntArray& qd_des, JntArray& torq_out ){
  // make sure that vector sizes are correct
  unsigned int jnt_count = FINGER_COUNT * FINGER_LENGTH;
  if (q.rows() != jnt_count || q_des.rows() != jnt_count ||
      qd_des.rows() != jnt_count )
  {
      ROS_ERROR("JointPositionController: joint arrays shall have size %d", jnt_count);
      return;
  }

  // virtual damping
  JntArray qd_des_damped(qd_des);
  if(damp_ratio_ != 0.0) applyVirtualDamping_(qd_des_damped);

  // Calc time since last control
  ros::Time t_now = ros::Time::now();
  double dt_update = (t_now - t_update_).toSec();
  // time init? too long delay? the memory variables are outdated
  if(dt_update > 0.5) {
    ROS_INFO("JointPositionController:\033[36m Re-initiated control state.\033[0m");
    dt_update = 0.0; // so the integral error will not explode
    q_last_.resize(0);
    resetIntegralError_();
  }
  // estimate the current velocity
  JntArray qd;
  estimateVelocity_(dt_update, q, qd);

  // clear torque vector
  torq_out.data = Eigen::VectorXd();

  // Repeat for each finger
  for(int fi=0; fi < FINGER_COUNT; fi++){

    // if finger is not active, then just pass
    if(!activateFinger_(fi, torq_out)) continue;

    // Calculate the torque using PID control
    Eigen::VectorXd t_pos =
      fingerSegment_(k_pos_vec_, fi).cwiseProduct(fingerSegment_(q_des, fi) - fingerSegment_(q, fi));
    Eigen::VectorXd t_vel =
      fingerSegment_(k_vel_vec_, fi).cwiseProduct(fingerSegment_(qd_des_damped, fi) - fingerSegment_(qd, fi));
    Eigen::VectorXd t_int =
      fingerSegment_(k_int_vec_, fi).cwiseProduct(fingerSegment_(e_sum_vec_, fi));

    // combine torque components
    Eigen::VectorXd t_finger = t_pos + t_vel + t_int;

    // append
    Eigen::VectorXd t_tmp(torq_out.data.rows()+ t_finger.rows());
    t_tmp << torq_out.data , t_finger;
    torq_out.data = t_tmp;

  }

  // accumulate intergal control error for future
  updateIntegralError_((q_des.data - q.data)*dt_update);


  t_update_ = ros::Time::now();
}
// wrapper with std vectors
void JointPositionController::computeTorques(const vector<double>& q, const vector<double>& q_des, const vector<double>& qd_des, vector<double>& torq_out )
{
  // create kdl data types
    // inputs
  JntArray q_kdl;
  JntArray q_des_kdl;
  JntArray qd_des_kdl;
    // output
  JntArray torq_kdl;

  // convert input to kdl
  kdl_control_tools::vectorStdToKdl(q, q_kdl);
  kdl_control_tools::vectorStdToKdl(q_des, q_des_kdl);
  kdl_control_tools::vectorStdToKdl(qd_des, qd_des_kdl);

  // call the implementation
  computeTorques(q_kdl, q_des_kdl, qd_des_kdl, torq_kdl);

  // convert output to std
  kdl_control_tools::vectorKdlToStd(torq_kdl, torq_out);
}
/*********************************************************************
* Check activation of finger, fill with 0.0 torques if inactive
*********************************************************************/
bool JointPositionController::activateFinger_(int finger_no, KDL::JntArray& torq_out)
{
  if(!active_fingers_[finger_no]) {
    // zeros
    Eigen::VectorXd t_zeros(FINGER_LENGTH);
    t_zeros.setZero();
    // append to torques
    Eigen::VectorXd t_tmp(torq_out.data.rows()+ t_zeros.rows());
    t_tmp << torq_out.data , t_zeros;
    torq_out.data = t_tmp;
  }

  return active_fingers_[finger_no];
}
/*********************************************************************
* Apply virtual damping by scaling the velocity
*********************************************************************/
void JointPositionController::applyVirtualDamping_(JntArray& qd)
{
  // velocity ratio is the inverse of damping ratio
  double vel_ratio = (1.0-damp_ratio_);

  unsigned int size = qd.rows();

  for(unsigned int qi =0; qi < size; qi++)
    qd(qi) = qd(qi) * vel_ratio;
}
/*********************************************************************
* adding of single dimension of error
* if the sign changed, the error resets
*********************************************************************/
double JointPositionController::addIntegralError_(const double e_total, const double e_last) {
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
void JointPositionController::updateIntegralError_(const Eigen::VectorXd& err){
  // add the last error to integral error
  for(int fi=0; fi < FINGER_COUNT; fi++){
    // update integral error only when the finger is active
    if(active_fingers_[fi]){
      for(int ji=fi*FINGER_LENGTH; ji < (fi+1)*FINGER_LENGTH; ji++)
        e_sum_vec_(ji) = addIntegralError_(e_sum_vec_(ji), err(ji));
    }
  }

}
/*********************************************************************
* Assigns zero to error vector
*********************************************************************/
void JointPositionController::resetIntegralError_(){
  for(int i=0; i< FINGER_COUNT*FINGER_LENGTH; i++)
    e_sum_vec_(i) = 0;
}
/*********************************************************************
* Use internal timer to estimate velocity since last control
*********************************************************************/
void JointPositionController::estimateVelocity_(const double dt_update, const JntArray& q, JntArray& qd)
{
  // if q_last_ doesn't exist
  if(q_last_.rows() != q.rows())
  {
    // zero velocity array
    qd = JntArray(q.rows());

  // if no time passed yet
  } else if(dt_update <= 1e-4)
  {
    ROS_WARN("JointPositionController: delta time is 0, multiple calls at the same moment!");
    qd.data = vel_past_;
    return;

  // otherwise, the normal case
  } else
  {
    // p_logger_->log("dt", dt_update);
    // differentiate q
    Eigen::VectorXd qd_eigen = (q.data - q_last_.data) / dt_update;

    // smooth
    qd_eigen = (vel_past_ *0.4 + qd_eigen *0.6);
    vel_past_ = qd_eigen;

    kdl_control_tools::arrayEigenToKdl(qd_eigen, qd);
  }
  // update state
  q_last_ = q;
}
/*********************************************************************
* Return the joint values corresponding to the given finger_no
*********************************************************************/
Eigen::VectorXd JointPositionController::fingerSegment_(const JntArray& arr, const int finger_no){
  return arr.data.segment(finger_no*FINGER_LENGTH, FINGER_LENGTH);
}
/*********************************************************************
* Getters and setters
*********************************************************************/
// setters
void JointPositionController::setGains(const double k_pos, const double k_vel, const double k_int)
{ setPositionGain(k_pos);
  setVelocityGain(k_vel);
  setIntegralGain(k_int); }
void JointPositionController::setGains(
  const vector<double>& k_pos_vec, const vector<double>& k_vel_vec, const vector<double>& k_int_vec)
{ setPositionGain(k_pos_vec);
  setVelocityGain(k_vel_vec);
  setIntegralGain(k_int_vec); }
void JointPositionController::setPositionGain(const double k_pos)
{ k_pos_vec_.resize(FINGER_LENGTH*FINGER_COUNT);
  for(uint i=0; i < k_pos_vec_.rows(); i++) k_pos_vec_(i) = k_pos; }
void JointPositionController::setPositionGain(const vector<double>& k_pos_vec)
{ kdl_control_tools::vectorStdToKdl(k_pos_vec, k_pos_vec_); }
void JointPositionController::setVelocityGain(const double k_vel)
{ k_vel_vec_.resize(FINGER_LENGTH*FINGER_COUNT);
  for(uint i=0; i < k_vel_vec_.rows(); i++) k_vel_vec_(i) = k_vel; }
void JointPositionController::setVelocityGain(const vector<double>& k_vel_vec)
{ kdl_control_tools::vectorStdToKdl(k_vel_vec, k_vel_vec_); }
void JointPositionController::setIntegralGain(const double k_int)
{ k_int_vec_.resize(FINGER_LENGTH*FINGER_COUNT);
  for(uint i=0; i < k_int_vec_.rows(); i++) k_int_vec_(i) = k_int; }
void JointPositionController::setIntegralGain(const vector<double>& k_int_vec)
{ kdl_control_tools::vectorStdToKdl(k_int_vec, k_int_vec_); }
void JointPositionController::setIntegralDecay(const double decay_rate)
{ decay_int_ = decay_rate; }
void JointPositionController::setDamping(const double damp_ratio)
{ damp_ratio_ = min(1.0, max(0.0, damp_ratio)); }
void JointPositionController::setActiveFingers(const int active_digits)
{
  // check digits by modulus
  for(int fi=0; fi<FINGER_COUNT; fi++){
    int digit_no = FINGER_COUNT-fi-1;
    int digit_val = pow(10, digit_no);
    active_fingers_[fi] = (active_digits)%(digit_val*10) >= digit_val;
  }
}
void JointPositionController::setActiveFingers(const vector<bool> activity_vec)
{ active_fingers_ = activity_vec; }
void JointPositionController::setActiveFingers(const vector<uint8_t> activity_vec)
{
  for(int fi=0; fi<FINGER_COUNT; fi++)
    active_fingers_[fi] = (bool) activity_vec[fi];
}
// getters
vector<double> JointPositionController::getPositionGain()
{ vector<double> ret_vec;
  kdl_control_tools::vectorKdlToStd(k_pos_vec_, ret_vec);
  return ret_vec; }
vector<double> JointPositionController::getVelocityGain()
{ vector<double> ret_vec;
  kdl_control_tools::vectorKdlToStd(k_vel_vec_, ret_vec);
  return ret_vec; }
vector<double> JointPositionController::getIntegralGain()
{ vector<double> ret_vec;
  kdl_control_tools::vectorKdlToStd(k_int_vec_, ret_vec); }
double JointPositionController::getIntegralDecay()
{ return decay_int_; }
double JointPositionController::getDamping()
{ return damp_ratio_; }
vector<bool> JointPositionController::getActiveFingers()
{ return active_fingers_; }

/*********************************************************************
* Connect an external ProgressLogger and log to it
*********************************************************************/
void JointPositionController::connectLogger(kdl_control_tools::ProgressLogger* logger){
  p_logger_ = logger;
}
