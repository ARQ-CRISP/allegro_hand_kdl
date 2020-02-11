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


#ifndef TEMPLATE_TRAJECTORY_CONTROL_H
#define TEMPLATE_TRAJECTORY_CONTROL_H

#include <memory>

#include <ros/ros.h>

#include <allegro_hand_kdl/cartesian_position_control.h>
#include <allegro_hand_kdl/joint_position_control.h>

#include <kdl_control_tools/trajectory.h>
#include <kdl_control_tools/multi_robot_trajectory.h>
#include <kdl_control_tools/progress_logger.h>

using namespace std;

namespace allegro_hand_kdl
{

// TemplateTrajectoryController of allegro_hand_kdl package contains
// the common interface that trajectory controller share.
template<class TrajType, class ControlType>
class TemplateTrajectoryController
{

  protected:

  public:
  	TemplateTrajectoryController(){
      traj = make_shared<TrajType>();
      control = make_shared<ControlType>();
    }
  	~TemplateTrajectoryController(){}

    virtual void computeTorques(const double t_current, const JntArray& q, JntArray& tau ) = 0;
    virtual void computeTorques(const double t_current, const vector<double>& q, vector<double>& tau ) = 0;

    virtual void inline setTrajectory(std::shared_ptr<TrajType> traj){
      this->traj = traj;
    }

    virtual void inline setController(std::shared_ptr<ControlType> control){
      this->control = control;
    }

    // obsolete, use kdl_control_tools::Trajectory::getIndex(double) instead
    int inline getCurrentStep(double t){

      ROS_WARN("TemplateTrajectoryController::getCurrentStep(double) is now obsolete. Use kdl_control_tools::Trajectory::getIndex(double) instead.");

      return traj->getIndex(t);
    }

    std::shared_ptr<TrajType> traj;
    std::shared_ptr<ControlType> control;

};

class JointTrajectoryController : public TemplateTrajectoryController<kdl_control_tools::JointTrajectory, JointPositionController>
{
  public:
    void computeTorques(const double t_current, const JntArray& q, JntArray& tau );
    void computeTorques(const double t_current, const vector<double>& q, vector<double>& tau );
};
class CartesianTrajectoryController : public TemplateTrajectoryController<kdl_control_tools::HandTrajectory, CartesianPositionController>
{
  public:
    void computeTorques(const double t_current, const JntArray& q, JntArray& tau );
    void computeTorques(const double t_current, const vector<double>& q, vector<double>& tau );

    void computeForces(const double t_current, const JntArray& q, HandAcceleration& f);
    void computeForces(const double t_current, const vector<double>& q, HandAcceleration& f);
};

} // end namespace allegro_hand_kdl

#endif // TEMPLATE_TRAJECTORY_CONTROL_H
