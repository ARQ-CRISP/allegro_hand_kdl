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

#include <allegro_hand_kdl/template_trajectory_control.h>

using namespace std;

namespace allegro_hand_kdl
{

  void CartesianTrajectoryController::computeTorques(const double t_current, const JntArray& q, JntArray& tau ){

    // index to access the current desired values
    int current_step = traj->getIndex(t_current);

    // create desired state vectors
    const HandPose x_des = traj->getPoint(current_step);
    const HandVelocity xd_des = traj->getVelocity(current_step);
    // JntArray qdd_des;

    // inform the controller about the current pose
    control->updatePose(q);
    tau.data = control->computeTorques(x_des, xd_des).data;
  }
  // std wrapper
  void CartesianTrajectoryController::computeTorques(const double t_current, const vector<double>& q, vector<double>& tau){
    // create kdl types
    JntArray q_kdl;
    JntArray tau_kdl;
    // convert to kdl types
    kdl_control_tools::vectorStdToKdl(q, q_kdl);
    kdl_control_tools::vectorStdToKdl(tau, tau_kdl);

    // call the core method
    this->computeTorques(t_current, q_kdl, tau_kdl);

    // convert back to std
    kdl_control_tools::vectorKdlToStd(tau_kdl, tau);

  }

  /**
    * returns cartesian forces instead of joint torques
   **/
  void CartesianTrajectoryController::computeForces(const double t_current, const JntArray& q, HandAcceleration& f){
    // index to access the current desired values
    int current_step = traj->getIndex(t_current);

    // create desired state vectors
    const HandPose x_des = traj->getPoint(current_step);
    const HandVelocity xd_des = traj->getVelocity(current_step);
    // JntArray qdd_des;

    // inform the controller about the current pose
    control->updatePose(q);
    f = control->computeForces(x_des, xd_des);
  }
  // std wrapper
  void CartesianTrajectoryController::computeForces(const double t_current, const vector<double>& q, HandAcceleration& f){
    // convert to kdl types
    JntArray q_kdl;
    kdl_control_tools::vectorStdToKdl(q, q_kdl);

    // call the core method
    this->computeForces(t_current, q_kdl, f);
  }

} // end namespace allegro_hand_kdl
