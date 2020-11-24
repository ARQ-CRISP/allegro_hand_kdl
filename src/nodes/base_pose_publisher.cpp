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

#include <signal.h>

#include <kdl_conversions/kdl_msg.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Pose.h>

/**
* This node will publish the current base frame pose of Allegro hand.
* It listens to tf publisher for the transform of hand_root
* with respect to base frame. This node is useful for controlling a
* mounted Allegro hand.
**/

using namespace std;

tf2_ros::Buffer tfBuffer;

ros::Publisher pub_; // publishes the hand_root transform as a Pose msg

double freq_pub_= 50;

// tf naming params
string robot_base_;
string hand_base_;

// functions
bool getParams();
void timerCallback(const ros::TimerEvent&);

int main(int argc, char **argv){

  ros::init(argc, argv, "base_pose_publisher");
  ROS_INFO("Allegro base pose publisher");

  ros::NodeHandle nh;

  // attempt to get ros parameters
  if(!getParams()) return -1;

  // create tf listener
  tf2_ros::TransformListener tfListener(tfBuffer);

  // create publishers
  pub_ =
      nh.advertise<geometry_msgs::Pose>("base_pose", 1);

  // start timer loop
  ros::Timer timer = nh.createTimer(ros::Duration(1/freq_pub_), timerCallback);

  ROS_WARN("OBSOLETE: Base pose publisher is not needed by gravity compensation anymore.");

  ros::spin();

  return 0;

}

// get external parameters if possible
bool getParams(){

  string param_name;
  //*********
  if(!ros::param::get("~robot_base", robot_base_) )
  {
    ROS_ERROR("Base pose publisher: Can't find robot_base param.");
    return -1;
  }
  ROS_INFO( "Base pose publisher: robot base frame: %s", robot_base_.c_str() );

  //*********
  if(!ros::param::get("~hand_base", hand_base_) )
  {
    ROS_ERROR("Base pose publisher: Can't find hand_base param.");
    return -1;
  }
  ROS_INFO( "Base pose publisher: hand base frame: %s", hand_base_.c_str() );

  return true;
}

// read the current transform and publish hand_root pose
void timerCallback(const ros::TimerEvent&)
{
  // get the transform info
  geometry_msgs::Transform tf_hand;

  try
  {
    // transform of hand root w.r.t. robot base
    tf_hand = tfBuffer.lookupTransform(
      robot_base_,
      hand_base_,
      ros::Time(0)).transform;
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s", ex.what());
    ros::Duration(1.0).sleep();
    return;
  }

  // publish it as a Pose msg
  geometry_msgs::Pose msg;

  msg.orientation = tf_hand.rotation;
  msg.position.x = tf_hand.translation.x;
  msg.position.y = tf_hand.translation.y;
  msg.position.z = tf_hand.translation.z;

  pub_.publish(msg);
}
