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


#ifndef CARTESIAN_FORCE_H
#define CARTESIAN_FORCE_H

#include <ros/ros.h>

#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>

#include <eigen3/Eigen/Dense>

#include <allegro_hand_kdl/allegro_kdl_config.h>
#include <kdl_control_tools/kdl_helper.h>

namespace allegro_hand_kdl
{

using std::vector;
// CarteisanForce of allegro_hand_kdl package uses the KDL jacobian solver to
// calculate the required joint torques to achieve desired cartesian forces.
class CartesianForce
{
  private:
    vector<ChainJntToJacSolver*> finger_solver_vec_;

    void createFingerSolvers_(AllegroKdlConfig& kdl_config);

  public:
    CartesianForce(AllegroKdlConfig& kdl_config);
    ~CartesianForce();

    // returns the joint torques which achieve the desired cartesian forces
    KDL::JntArray computeTorques(const KDL::JntArray& q, const vector<KDL::JntArray>& f_cart);
    KDL::JntArray computeTorques(const KDL::JntArray& q, const vector<KDL::Wrench>& f_cart);
    vector<double> computeTorques(const vector<double>& q, const vector< vector<double> >& f_cart);
    // returns the cartesian forces resulting from given joint torques
    vector<KDL::Wrench> computeWrenches(const KDL::JntArray& q, const KDL::JntArray& tau);
};

}  // end namespace allegro_hand_kdl

#endif // CARTESIAN_FORCE_H
