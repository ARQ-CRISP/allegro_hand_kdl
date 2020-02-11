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


#ifndef INVERSE_DYNAMICS_H
#define INVERSE_DYNAMICS_H

#include <ros/ros.h>

#include <kdl/chain.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/chaindynparam.hpp>

#include <eigen3/Eigen/Core>

#include <allegro_hand_kdl/allegro_kdl_config.h>

using namespace std;
using namespace KDL;

namespace allegro_hand_kdl
{

// InverseDynamics of allegro_hand_kdl package is a wrapper for
// Orocos KDL's inverse dynamics solver ChainIdSolver_RNE.
// It creates appropriate solvers for each finger using the Allegro hand description.
// Then solvers are used to return the set of torque values for each joint,
// in order to achieve a desired joint-space motion (position, velocity, acceleration).
class InverseDynamics
{

  private:

    vector<ChainIdSolver_RNE*> finger_solver_vec_;
    vector<ChainDynParam*> finger_params_vec_;

    void createFingerSolvers_(AllegroKdlConfig& kdl_config);
    void clearFingerSolvers_();


  public:
  	InverseDynamics(AllegroKdlConfig& kdl_config);
  	~InverseDynamics();

    void updateChains(AllegroKdlConfig& kdl_config);

    void computeTorques(const vector<double>& q, const vector<double>& qd, const vector<double>& qdd, vector<double>& tau );
    void computeTorquesFromWrenches(const vector<double>& q, const vector<double>& qd, const vector<double>& qdd, const vector<vector<double> >& wrenches, vector<double>& tau );

    void getGravityParam(const vector<double>& q, vector<double>& gravity );
    void getCoriolisParam(const vector<double>& q, const vector<double>& qd, vector<double>& coriolis );
    void getMassParam(const vector<double>& q, vector<vector<double> >& inertia);

    void getGravityParam_kdl(const int finger_index, const JntArray& q, JntArray& gravity );
    void getCoriolisParam_kdl(const int finger_index, const JntArray& q, const JntArray& qd, JntArray& coriolis );
    void getMassParam_kdl(const int finger_index, const JntArray& q, JntSpaceInertiaMatrix& inertia);

};

} // end namespace allegro_hand_kdl

#endif // INVERSE_DYNAMICS_H
