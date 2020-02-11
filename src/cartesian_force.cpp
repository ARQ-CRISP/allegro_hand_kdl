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

#include "allegro_hand_kdl/cartesian_force.h"


using namespace std;
using namespace KDL;

namespace allegro_hand_kdl
{
/*********************************************************************
* Creates solvers for each finger
*********************************************************************/
CartesianForce::CartesianForce(AllegroKdlConfig& kdl_config){

  if(!kdl_config.isReady()){
    ROS_ERROR("CartesianForce: null kdl config");
    ros::shutdown();
    return;
  }

  createFingerSolvers_(kdl_config);
}
CartesianForce::~CartesianForce(){

  for (ChainJntToJacSolver* obj : finger_solver_vec_){
    delete obj;
  }
  finger_solver_vec_.clear();
}

/*********************************************************************
* Creates solvers for each finger
*********************************************************************/
void CartesianForce::createFingerSolvers_(AllegroKdlConfig& kdl_config)
{

  // Create inverse dynamics solvers for each finger
  for(int fi=0; fi < FINGER_COUNT; fi++){

    // Extract the finger chain from whole hand tree
    Chain *finger_chain = kdl_config.getFingerChain(fi);

    // Solver requires the chain and the gravity info
    ChainJntToJacSolver* solver = new ChainJntToJacSolver(*finger_chain);

    // Fill vectors
    finger_solver_vec_.push_back(solver);

  }
}

/*********************************************************************
* Calculates the joint torques which achieve the desired cartesian forces.
*********************************************************************/
KDL::JntArray CartesianForce::computeTorques(const KDL::JntArray& q, const vector<KDL::JntArray>& w_vec){

  // make sure that vector sizes are correct
  unsigned int jnt_count = FINGER_COUNT * FINGER_LENGTH;
  if(w_vec.size() != FINGER_COUNT || w_vec[0].rows() != 6 || q.rows() != jnt_count){
    ROS_ERROR("CartesianForce: input lengths are wrong.");
    return KDL::JntArray();
  }

  // empty torque vector
  KDL::JntArray t_joint;

  // Repeat for each finger
  for(int fi=0; fi < FINGER_COUNT; fi++){

    // Create KDL data types
    KDL::Jacobian jac(FINGER_LENGTH);

    // Slice the finger joint angles
    KDL::JntArray q_fi;
    q_fi.data = q.data.segment(fi*FINGER_LENGTH, FINGER_LENGTH);

    // Get the jacobian for desired configuration
    int error = finger_solver_vec_[fi]->JntToJac(q_fi, jac); // 4 is the index of tip segment in finger chain
    if(error == -100)
      ROS_ERROR("Cartesian force: failed to derive Jacobian, finger %d.",fi);

    // Calculate the torque using jacobian
    Eigen::VectorXd t_finger = jac.data.transpose() * w_vec[fi].data;

    // append
    Eigen::VectorXd t_tmp(t_joint.data.rows()+ t_finger.rows());
    t_tmp << t_joint.data , t_finger;
    t_joint.data = t_tmp;

  }

  return t_joint;
}
// wrapper for std and ros types
// f_cart shall have size 4x6 (fingers count:4, wrench: 6)
vector<double> CartesianForce::computeTorques(const vector<double>& q, const vector< vector<double> >& f_cart){

  // Create KDL data types
    // inputs
  KDL::JntArray q_kdl;
  vector<KDL::JntArray> f_kdl(FINGER_COUNT);
    // output
  KDL::JntArray t_kdl;

  // convert input to kdl
  kdl_control_tools::vectorStdToKdl(q, q_kdl);
  for(int fi=0; fi < FINGER_COUNT; fi++)
    kdl_control_tools::vectorStdToKdl(f_cart[fi], f_kdl[fi]);

  // call the core method
  t_kdl = computeTorques(q_kdl, f_kdl);

  // convert the output to desired type
  vector<double> t_std;
  kdl_control_tools::vectorKdlToStd(t_kdl, t_std);

  return t_std;
}

} // end of namespace allegro_hand_kdl
