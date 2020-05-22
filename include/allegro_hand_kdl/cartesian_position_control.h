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


#ifndef CARTESIAN_POSITION_CONTROL_H
#define CARTESIAN_POSITION_CONTROL_H

#include <ros/ros.h>

#include <eigen3/Eigen/Dense>

#include <allegro_hand_kdl/cartesian_force.h>
#include <allegro_hand_kdl/kinematics.h>
#include <kdl_control_tools/kdl_helper.h>

#include <kdl_conversions/kdl_msg.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

using namespace std;
using namespace KDL;

namespace allegro_hand_kdl
{

typedef vector<KDL::Frame> HandPose;
typedef vector<KDL::Twist> HandVelocity;
typedef vector<KDL::Wrench> HandAcceleration;

/* CartesianPositionController of allegro_hand_kdl package calculates the
 * required torques for converging to desired fingertip cartesian positions
 * and velocities, using PID control and manipulator Jacobians.
 */
class CartesianPositionController
{

  private:

    // cartesian force solver to realize fingertip forces
    unique_ptr<CartesianForce> cf_solver_;
    unique_ptr<Kinematics> kin_solver_;

    // finger activation
    vector<bool> active_fingers_;

    // state memory
    JntArray q_cur_;
    HandPose x_cur_;
    HandPose x_last_;
    HandVelocity vel_past_;

    // control gain params
    // position
    double k_p_;
    double k_r_;
    double k_d_;
    double k_i_;

    double decay_int_;
    vector<KDL::Twist > e_sum_vec_; // sum of past error

    // timing
    ros::Time t_update_;

    void initParams_();
    double addIntegralError_(const double e_total, const double e_last);
    void updateIntegralError_(const HandPose& x, const HandPose& x_des, double t);
    void estimateVelocity_(const double dt_update, const HandPose& x, HandVelocity& xd);
    bool checkKdlObjects_();

  public:
    CartesianPositionController();
  	CartesianPositionController(AllegroKdlConfig& kdl_config);
  	~CartesianPositionController();

    void createKdlObjects(AllegroKdlConfig& kdl_config);

    HandAcceleration computeForces(const HandPose& x_des, const HandVelocity& xd_des);

    KDL::JntArray computeTorques(const HandPose& x_des, const HandVelocity& xd_des);
    vector<double> computeTorques(const vector<geometry_msgs::Pose>& x_des, const vector<geometry_msgs::Twist>& xd_des);

    HandPose updatePose(const JntArray& q);
    HandPose updatePose(const vector<double>& q);
    void updatePose(const HandPose& x);

    void resetIntegralError();

    // Getters and setters
    void setGains(const double k_pos, const double k_rot, const double k_vel, const double k_int);
    void setPositionGain(const double k_pos);
    void setRotationGain(const double k_rot);
    void setVelocityGain(const double k_vel);

    void setIntegralGain(const double k_int);
    void setIntegralDecay(const double decay_rate);

    void setActiveFingers(const int active_digits);
    void setActiveFingers(const vector<bool> activity_vec);
    void setActiveFingers(const vector<uint8_t> activity_vec);

    double getPositionGain();
    double getRotationGain();
    double getVelocityGain();
    double getIntegralGain();
    double getIntegralDecay();
    vector<bool> getActiveFingers();

};

} // end namespace allegro_hand_kdl

#endif // CARTESIAN_POSITION_CONTROL_H
