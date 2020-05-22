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


#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <random>

#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include <kdl/chain.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <eigen3/Eigen/Core>

#include <allegro_hand_kdl/allegro_kdl_config.h>

#include <kdl_control_tools/kdl_helper.h>

using namespace std;
using namespace KDL;

namespace allegro_hand_kdl
{

// Kinematics of allegro_hand_kdl package is a wrapper for
// Orocos KDL's forward and inverse kinematics solvers, both for position and velocity.
// It creates appropriate solvers for each finger using the Allegro hand description.
// Then solvers are used to return the positions and velocities for each joint,
// in order to achieve a desired cartesian-space position and velocity.
// chainfksolverpos_recursive, chainiksolverpos_nr_jl
// and chainiksolvervel_pinv implementations are used.
class Kinematics
{

  private:

    vector<ChainFkSolverPos_recursive*> finger_fk_pos_;
    vector<ChainIkSolverVel_pinv*> finger_ik_vel_;
    vector<ChainIkSolverPos*> finger_ik_pos_;

    int perturbed_trials_;
    bool warned_perturbed_trials_;

    void createFingerSolvers_(AllegroKdlConfig& kdl_config, const vector<double>& q_min, const vector<double>& q_max, unsigned int maxiter, double eps);
    KDL::JntArray perturbJointAngles_(const KDL::JntArray& q, double scale=0.05);
    int attemptIK_(int finger_index, const JntArray& q_init,
                const KDL::Frame& x_des, JntArray& q_des, int trial=0);

    std::default_random_engine rand_;

  public:
    /**
      * If perturbed_trials is greater than zero, solving attempts are repeated
      * with perturbed initial states when the solution is inferior.
      * This would increase the solving time, but may obtain better results.
    **/
    Kinematics(AllegroKdlConfig& kdl_config, int perturbed_trials=0,
        unsigned int maxiter = 300, double eps = 1e-5);
    // TODO: remove joint limited IK (ChainIkSolverPos_NR_JL), as it does not work well
    Kinematics(AllegroKdlConfig& kdl_config,
        const vector<double>& q_min, const vector<double>& q_max,
        int perturbed_trials=0, unsigned int maxiter = 300,
        double eps = 1e-5);
  	~Kinematics();
    
    void toJointSpace(const vector<double>& q_init, const vector<geometry_msgs::Pose>& x_des, const vector<geometry_msgs::Twist>& xd_des,
          vector<double>& q_des, vector<double>& qd_des);
    void toJointSpace(const JntArray& q_init, const vector<KDL::Frame>& x_des, const vector<KDL::Twist>& xd_des,
          JntArray& q_des, JntArray& qd_des);

    int calcJointPos(const JntArray& q_init, const vector<KDL::Frame>& x_des, JntArray& q_des);
    int calcJointPos(const vector<double>& q_init, const vector<geometry_msgs::Pose>& x_des, vector<double>& q_des);

    void calcJointVel(const JntArray& q_init, const vector<KDL::Twist>& xd_des, JntArray& qd_des);
    void calcJointVel(const vector<double>& q_init, const vector<geometry_msgs::Twist>& xd_des, vector<double>& qd_des);

    void calcCartPos(const JntArray& q, vector<KDL::Frame>& x);
    void calcCartPos(const vector<double>& q, vector<KDL::Frame>& x);
    void calcCartPos(const vector<double>& q, vector<geometry_msgs::Pose>& x);
};

} // end namespace allegro_hand_kdl

#endif // KINEMATICS_H
