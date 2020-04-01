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


#ifndef JOINT_POSITION_CONTROL_H
#define JOINT_POSITION_CONTROL_H

#include <ros/ros.h>

#include <allegro_hand_kdl/allegro_kdl_config.h>
#include <kdl_control_tools/kdl_helper.h>

#include <kdl_control_tools/progress_logger.h>

using namespace std;
using namespace KDL;

namespace allegro_hand_kdl
{

/* JointPositionController of allegro_hand_kdl package calculates the
 * required torques for converging to desired joint positions and velocities.
 * It uses PID control.
 */
class JointPositionController
{

  private:

    // finger activation
    vector<bool> active_fingers_;

    // state memory
    JntArray q_last_;
    Eigen::VectorXd vel_past_;

    // PD gain vectors
    // each joint may have its own gain
    JntArray k_pos_vec_;
    JntArray k_vel_vec_;

    // Integral control
    JntArray k_int_vec_;
    double decay_int_;
    JntArray e_sum_vec_; // sum of past error

    // virtual damping in [0,1)
    double damp_ratio_;

    // timing
    ros::Time t_update_;

    // debug mode variables
    kdl_control_tools::ProgressLogger * p_logger_;

    bool activateFinger_(int finger_no, KDL::JntArray& torq_out);
    void applyVirtualDamping_(JntArray& qd);
    double addIntegralError_(const double e_total, const double e_last);
    void updateIntegralError_(const Eigen::VectorXd& err);
    void resetIntegralError_();
    void estimateVelocity_(const double dt_update, const JntArray& q, JntArray& qd);
    Eigen::VectorXd fingerSegment_(const JntArray& q, const int finger_no);

  public:
  	JointPositionController();
  	~JointPositionController();

    void computeTorques(const KDL::JntArray& q, const KDL::JntArray& q_des, const KDL::JntArray& qd_des, KDL::JntArray& torq_out );
    void computeTorques(const vector<double>& q, const vector<double>& q_des, const vector<double>& qd_des, vector<double>& torq_out );

    // Getters and setters
    void setGains(const double k_pos, const double k_vel, const double k_int);
    void setGains(const vector<double>& k_pos_vec,
                  const vector<double>& k_vel_vec,
                  const vector<double>& k_int_vec);
    void setPositionGain(const double k_pos);
    void setPositionGain(const vector<double>& k_pos_vec);
    void setVelocityGain(const double k_vel);
    void setVelocityGain(const vector<double>& k_vel_vec);
    void setIntegralGain(const double k_int);
    void setIntegralGain(const vector<double>& k_int_vec);
    void setIntegralDecay(const double decay_rate);
    void setDamping(const double damp_ratio);
    void setActiveFingers(const int active_digits);
    void setActiveFingers(const vector<bool> activity_vec);
    void setActiveFingers(const vector<uint8_t> activity_vec);

    vector<double> getPositionGain();
    vector<double> getVelocityGain();
    vector<double> getIntegralGain();
    double getIntegralDecay();
    double getDamping();
    vector<bool> getActiveFingers();

    // debug functions
    void connectLogger(kdl_control_tools::ProgressLogger* logger);

};

} // end namespace allegro_hand_kdl

#endif // JOINT_POSITION_CONTROL_H
