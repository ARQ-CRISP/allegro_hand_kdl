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


#ifndef ALLEGRO_KDL_CONFIG_H
#define ALLEGRO_KDL_CONFIG_H

#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>

#include <kdl_conversions/kdl_msg.h>
#include <geometry_msgs/Quaternion.h>

using namespace std;
using namespace KDL;

namespace allegro_hand_kdl
{

const int FINGER_COUNT = 4; // How many fingers are there
const int FINGER_LENGTH = 4; // How many segments has each finger

// AllegroKdlConfig of allegro_hand_kdl package is a utility to
// parse KDL tree from Allegro hand urdf file and hold it to be
// accessed by solver objects.
class AllegroKdlConfig
{

  private:

    vector<Chain*> finger_chain_vec_;
    Tree kdl_tree_;

    bool ready_;

    void createFingerChains_(const string root_frame);


  public:
  	AllegroKdlConfig();
  	~AllegroKdlConfig();

    // parse the internal kdl structures using a string, file or ros handle.
    void parseKdl(const ros::NodeHandle& node, const string root_frame="hand_root");
    void parseKdl(const string& str_description, bool from_file, const string root_frame="hand_root");

    // modify the root orientation of kdl chains (useful with mounting)
    void rotateBase(const geometry_msgs::Quaternion& rot);
    void rotateBase(const KDL::Rotation& rot);

    // accessors
    Chain* getFingerChain(const int index) const;
    bool isReady();

};

} // end namespace allegro_hand_kdl

#endif // ALLEGRO_KDL_CONFIG_H
