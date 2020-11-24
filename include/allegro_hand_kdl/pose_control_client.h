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


#ifndef POSE_CONTROL_CLIENT_H
#define POSE_CONTROL_CLIENT_H

#include <ros/ros.h>

#include <allegro_hand_kdl/allegro_kdl_config.h>
#include <allegro_hand_kdl/PoseRequest.h>

using namespace std;
//
namespace allegro_hand_kdl
{

/* Pose control client is a class that handles communication
 * with a pose server.
 */
template<class PoseType>
class PoseControlClient
{

  protected:
    ros::NodeHandle nh_;
    ros::ServiceClient srv_;

    string target_name_;
    string behaviour_;
    vector<uint8_t> active_fingers_;

    inline int findPose_(string ns, string name)
    {
      // iterate existing poses (following pattern p0,p1...)
      int pi = 0;
      while (ros::param::has(ns+"/p"+to_string(pi)))
      {
        // compare the existing names to the searched name
        string current_name;
        ros::param::get(ns+"/p"+to_string(pi)+"/name", current_name);
        if (current_name == name)
          return pi;
        pi++;
      }
      // if no name is found, return the last index
      return pi;
    }

  public:
    inline PoseControlClient(const string& srv_name, const string& behaviour=""): behaviour_(behaviour)
    {
      nh_ = ros::NodeHandle();
      srv_ =
        nh_.serviceClient<allegro_hand_kdl::PoseRequest>(srv_name);
    }
  	inline ~PoseControlClient(){};

    // create new pose and (over)write on the name
    virtual void setTargetPose(vector<PoseType>& pose_vec, string name="tmp") = 0;

    // existing pose
    void setTargetName(string name){
      target_name_ = name;
    }

    void setBehaviour(const string& behaviour){
      behaviour_ = behaviour;
    }

    void setActiveFingers(const vector<bool>& active_fingers){
      for(int fi=0; fi<FINGER_COUNT; fi++)
        active_fingers_[fi] = (bool) active_fingers[fi];
    }

    void setActiveFingers(const vector<uint8_t>& active_fingers)
    {
      active_fingers_ = active_fingers;
    }

    void activateAllFingers()
    {
      active_fingers_ = vector<uint8_t>(FINGER_COUNT, 1);
    }

    // keeps its current pose
    inline bool maintainPose()
    {
      setTargetName("");
      move();
    }

    // just calls the server to move
    inline bool move()
    {
      allegro_hand_kdl::PoseRequest req;
      req.request.pose = target_name_;
      req.request.behaviour = behaviour_;
      req.request.active_fingers = active_fingers_;

      return srv_.call(req);
    }

};

class CartesianPoseClient : public PoseControlClient<geometry_msgs::Pose>
{
  public:
    inline CartesianPoseClient(const string& srv_name="desired_cartesian_pose", const string& behaviour="") :
      PoseControlClient(srv_name, behaviour){}

    // create new pose and (over)write on the name
    inline void setTargetPose(vector<geometry_msgs::Pose>& pose_vec, string name="tmp") override {
      // search for an existing pose with this name
      string pose_param_ns = "allegro_kdl/poses/cartesian_poses";
      int pi = this->findPose_(pose_param_ns, name);
      // override/create param with the new values
      ros::param::set(pose_param_ns+"/p"+to_string(pi)+"/name", name);
      // set states for each finger
      for (int fi=0; fi < pose_vec.size(); fi++)
      {
        string pose_path =
            pose_param_ns+"/p"+to_string(pi)+"/state/f"+to_string(fi);
        // position
        ros::param::set(pose_path+"/pos/x", pose_vec[fi].position.x);
        ros::param::set(pose_path+"/pos/y", pose_vec[fi].position.y);
        ros::param::set(pose_path+"/pos/z", pose_vec[fi].position.z);
        // orientation
        ros::param::set(pose_path+"/rot/x", pose_vec[fi].orientation.x);
        ros::param::set(pose_path+"/rot/y", pose_vec[fi].orientation.y);
        ros::param::set(pose_path+"/rot/z", pose_vec[fi].orientation.z);
        ros::param::set(pose_path+"/rot/w", pose_vec[fi].orientation.w);
      }
      // set the target pose name for execution
      PoseControlClient::setTargetName(name);
    }
};

class JointPoseClient : public PoseControlClient<double>
{
  public:
    inline JointPoseClient(const string& srv_name="desired_pose", const string& behaviour="") :
      PoseControlClient(srv_name, behaviour){}

    // create new pose and (over)write on the name
    inline void setTargetPose(vector<double>& pose_vec, string name="tmp") override {
      // search for an existing pose with this name
      string pose_param_ns = "allegro_kdl/poses/joint_poses";
      int pi = this->findPose_(pose_param_ns, name);
      // override/create param with the new values
      ros::param::set(pose_param_ns+"/p"+to_string(pi)+"/name", name);
      // set states for joints
      ros::param::set(pose_param_ns+"/p"+to_string(pi)+"/state", pose_vec);
      // set the target pose name for execution
      PoseControlClient::setTargetName(name);
    }
};

} // end namespace allegro_hand_kdl

#endif // POSE_CONTROL_CLIENT_H
