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

#include "allegro_hand_kdl/inverse_dynamics.h"

using namespace std;
using namespace KDL;

namespace allegro_hand_kdl
{
/*********************************************************************
* Creates solvers and dynamic params for each finger
*********************************************************************/
InverseDynamics::InverseDynamics(AllegroKdlConfig& kdl_config){

  if(!kdl_config.isReady()){
    ROS_ERROR("InverseDynamics: empty kdl config");
    ros::shutdown();
    return;
  }

  createFingerSolvers_(kdl_config);

}
/*********************************************************************
* Comment
*********************************************************************/
InverseDynamics::~InverseDynamics()
{
}

/*********************************************************************
* Creates solvers and dynamic params for each finger
*********************************************************************/
void InverseDynamics::createFingerSolvers_(AllegroKdlConfig& kdl_config)
{
  // Create default gravity vector
  KDL::Vector grav_vec(0.0,0.0,-9.8);

  // Create inverse dynamics solvers for each finger
  for(int fi=0; fi < FINGER_COUNT; fi++){

    // Extract the finger chain from whole hand tree
    Chain *finger_chain = kdl_config.getFingerChain(fi);

    // Solver requires the chain and the gravity info
    ChainIdSolver_RNE* solver = new ChainIdSolver_RNE(*finger_chain, grav_vec);

    // Create dynamics parameters for the finger (Alternative to solver)
    ChainDynParam* dynamicParam = new ChainDynParam(*finger_chain, grav_vec);

    // Fill vectors
    finger_solver_vec_.push_back(solver);
    finger_params_vec_.push_back(dynamicParam);

  }
}
/*********************************************************************
* Delete solvers and dynamic params of each finger
*********************************************************************/
void InverseDynamics::clearFingerSolvers_(){

  for(int fi=0; fi < FINGER_COUNT; fi++){

    delete finger_solver_vec_[fi];
    delete finger_params_vec_[fi];
  }

  // empty vectors
  finger_solver_vec_.clear();
  finger_params_vec_.clear();
}
/*********************************************************************
* Re-create solvers and dynamic params for each finger
*********************************************************************/
void InverseDynamics::updateChains(AllegroKdlConfig& kdl_config){

  if(!kdl_config.isReady()){
    ROS_ERROR("InverseDynamics: empty kdl config");
    ros::shutdown();
    return;
  }

  clearFingerSolvers_();
  createFingerSolvers_(kdl_config);
}
/*********************************************************************
* Calculates the torque for desired motion, using inverse dynamics params
*********************************************************************/
void InverseDynamics::computeTorques(const vector<double>& q, const vector<double>& qd, const vector<double>& qdd, vector<double>& tau )
{
  // make sure that tau vector is empty
  tau.clear();

  // Repeat for each finger
  for(int fi=0; fi < FINGER_COUNT; fi++){

    // Create KDL data types
      // inputs
    JntArray q_kdl(FINGER_LENGTH);
    JntArray qd_kdl(FINGER_LENGTH);
    JntArray qdd_kdl(FINGER_LENGTH);
      // ID parameters
    JntArray gravity_kdl(FINGER_LENGTH);
    JntArray coriolis_kdl(FINGER_LENGTH);
    JntSpaceInertiaMatrix inertia_kdl(FINGER_LENGTH);
      // output
    JntArray tau_kdl(FINGER_LENGTH);

    // Copy inputs from arguments
    for(int si=0; si<FINGER_LENGTH; si++){ // si = segment index
      q_kdl(si) = q[fi * FINGER_LENGTH + si];
      qd_kdl(si) = qd[fi * FINGER_LENGTH + si];
      qdd_kdl(si) = qdd[fi * FINGER_LENGTH + si];
    }

    // Get the inverse dynamics parameters for desired configuration/motion
    finger_params_vec_[fi]->JntToGravity(q_kdl, gravity_kdl);
    finger_params_vec_[fi]->JntToCoriolis(q_kdl, qd_kdl, coriolis_kdl);
    finger_params_vec_[fi]->JntToMass(q_kdl, inertia_kdl);

    // Calculate the torque using inverse dynamics motion equation
    // using Eigen data types
    Eigen::VectorXd t_ =
      gravity_kdl.data +
      coriolis_kdl.data.cwiseProduct(qd_kdl.data) +
      inertia_kdl.data * qdd_kdl.data;

    // Add output to std vector
    for(int si=0; si<FINGER_LENGTH; si++){ // si = segment index
      tau.push_back(t_(si));
    }

  }

}

/*********************************************************************
* Wrapper for ChainIdSolver_RNE::CartToJnt(...)
* 'wrenches' should have the size (finger_count*segment_count, 6)
* where 6 is the wrench size (3 from force, 3 from torque).
*********************************************************************/
void InverseDynamics::computeTorquesFromWrenches(const vector<double>& q, const vector<double>& qd, const vector<double>& qdd, const vector<vector<double> >& wrenches, vector<double>& tau )
{
  // make sure that tau vector is empty
  tau.clear();

  // Repeat for each finger
  for(int fi=0; fi < FINGER_COUNT; fi++){

    // Create KDL data types
    JntArray q_kdl(FINGER_LENGTH);
    JntArray qd_kdl(FINGER_LENGTH);
    JntArray qdd_kdl(FINGER_LENGTH);
    Wrenches wrenches_kdl;
    JntArray tau_kdl(FINGER_LENGTH);

    for(int si=0; si<FINGER_LENGTH; si++){ // si = segment index
      q_kdl(si) = q[fi * FINGER_LENGTH + si];
      qd_kdl(si) = qd[fi * FINGER_LENGTH + si];
      qdd_kdl(si) = qdd[fi * FINGER_LENGTH + si];
    }

    // Wrenches for each segment in the chain
    for(int si=0; si<5; si++){ // si = segment index
      Wrench w_segment;
      // Copy from input
      for(int wi=0; wi<6; wi++)
        w_segment(wi) = wrenches[fi * FINGER_LENGTH + si][wi];

      wrenches_kdl.push_back(w_segment);
    }

    // Call the solver
    finger_solver_vec_[fi]->CartToJnt(q_kdl, qd_kdl, qdd_kdl, wrenches_kdl, tau_kdl);

    // Add output to std vector
    for(int si=0; si<FINGER_LENGTH; si++){ // si = segment index
      tau.push_back(tau_kdl(si));
    }

  }

}

/*********************************************************************
* Wrapper for ChainDynParam::JntToGravity(...)
* returns data in std vector format
* for all fingers
*********************************************************************/
void InverseDynamics::getGravityParam(const vector<double>& q, vector<double>& gravity )
{
  // make sure that output vector is empty
  gravity.clear();

  // Repeat for each finger
  for(int fi=0; fi < FINGER_COUNT; fi++){

    // Create KDL data types
    JntArray q_kdl(FINGER_LENGTH);
    JntArray gravity_kdl(FINGER_LENGTH);

    for(int si=0; si<FINGER_LENGTH; si++){ // si = segment index
      q_kdl(si) = q[fi * FINGER_LENGTH + si];
    }

    // Get the param
    finger_params_vec_[fi]->JntToGravity(q_kdl, gravity_kdl);

    // Add output to std vector
    for(int si=0; si<FINGER_LENGTH; si++){ // si = segment index
      gravity.push_back(gravity_kdl(si));
    }

  }

}

/*********************************************************************
* Wrapper for ChainDynParam::JntToCoriolis(...)
* returns data in std vector format
* for all fingers
*********************************************************************/
void InverseDynamics::getCoriolisParam(const vector<double>& q, const vector<double>& qd, vector<double>& coriolis )
{
  // make sure that output vector is empty
  coriolis.clear();

  // Repeat for each finger
  for(int fi=0; fi < FINGER_COUNT; fi++){

    // Create KDL data types
    JntArray q_kdl(FINGER_LENGTH);
    JntArray qd_kdl(FINGER_LENGTH);
    JntArray coriolis_kdl(FINGER_LENGTH);

    for(int si=0; si<FINGER_LENGTH; si++){ // si = segment index
      q_kdl(si) = q[fi * FINGER_LENGTH + si];
      qd_kdl(si) = qd[fi * FINGER_LENGTH + si];
    }

    // Get the param
    finger_params_vec_[fi]->JntToCoriolis(q_kdl, qd_kdl, coriolis_kdl);

    // Add output to std vector
    for(int si=0; si<FINGER_LENGTH; si++){ // si = segment index
      coriolis.push_back(coriolis_kdl(si));
    }

  }

}


/*********************************************************************
* Wrapper for ChainDynParam::JntToMass(...)
* returns data in std vector format
* for all fingers
*********************************************************************/
void InverseDynamics::getMassParam(const vector<double>& q, vector<vector<double> >& inertia)
{
  // make sure that output vector is empty
  inertia.clear();

  // Repeat for each finger
  for(int fi=0; fi < FINGER_COUNT; fi++){

    // Create KDL data types
    JntArray q_kdl(FINGER_LENGTH);
    JntSpaceInertiaMatrix inertia_kdl(FINGER_LENGTH);

    for(int si=0; si<FINGER_LENGTH; si++){ // si = segment index
      q_kdl(si) = q[fi * FINGER_LENGTH + si];
    }

    // Get the param
    finger_params_vec_[fi]->JntToMass(q_kdl, inertia_kdl);

    // Add output to std vector
    // It will create a JOINT_COUNT x FINGER_LENGTH matrix (16 x 4)
    for(int si=0; si<FINGER_LENGTH; si++){ // si = segment index
      // create row
      inertia.push_back(vector<double>());

      for(int sj=0; sj<FINGER_LENGTH; sj++){
        int joint_index = fi * FINGER_LENGTH + si;
        inertia[ joint_index].push_back(inertia_kdl(si, sj));
      }
    }

  }

}

/*********************************************************************
* Wrapper for ChainDynParam::JntToGravity(...)
* returns data in kdl format
* for a single finger
*********************************************************************/
void InverseDynamics::getGravityParam_kdl(const int finger_index, const JntArray& q, JntArray& gravity )
{
    // Get the param
    finger_params_vec_[finger_index]->JntToGravity(q, gravity);
}
/*********************************************************************
* Wrapper for ChainDynParam::JntToCoriolis(...)
* returns data in kdl format
* for a single finger
*********************************************************************/
void InverseDynamics::getCoriolisParam_kdl(const int finger_index, const JntArray& q, const JntArray& qd, JntArray& coriolis )
{
    // Get the param
    finger_params_vec_[finger_index]->JntToCoriolis(q, qd, coriolis);
}
/*********************************************************************
* Wrapper for ChainDynParam::JntToMass(...)
* returns data in kdl format
* for a single finger
*********************************************************************/
void InverseDynamics::getMassParam_kdl(const int finger_index, const JntArray& q, JntSpaceInertiaMatrix& inertia)
{
    // Get the param
    finger_params_vec_[finger_index]->JntToMass(q, inertia);
}

} // end namespace
