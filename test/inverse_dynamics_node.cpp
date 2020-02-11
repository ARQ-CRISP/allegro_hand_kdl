#include<allegro_hand_kdl/inverse_dynamics.h>

#include <sensor_msgs/JointState.h>

using namespace std;
using namespace KDL;

allegro_hand_kdl::AllegroKdlConfig allegro_kdl_config;
allegro_hand_kdl::InverseDynamics* id_solver;


void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg) {

  ROS_INFO("InverseDynamics jointStateCallback");

  vector<double> joint_pos_vec_(16);

  // update the state
  for(int j=0; j<16; j++)
    joint_pos_vec_[j] = msg->position[j];

  // Calculate inverse dynamics torques for gravity compensation
  vector<double> torque_compensate;
  vector<double> q_zeros(16);
  id_solver->computeTorques(joint_pos_vec_, q_zeros, q_zeros, torque_compensate);

  // Test gravity params
  vector<double> gravity_param;
  vector<double> coriolis_param;
  vector<vector<double> > inertia_param;
  id_solver->getGravityParam(joint_pos_vec_, gravity_param);
  id_solver->getCoriolisParam(joint_pos_vec_, q_zeros, coriolis_param);
  id_solver->getMassParam(joint_pos_vec_, inertia_param);

  for(int j=0; j<16; j++)
    ROS_INFO("InverseDynamics torque %d, %f", j, torque_compensate[j]);

  for(int j=0; j<16; j++)
    ROS_INFO("InverseDynamics gravity param %d, %f", j, gravity_param[j]);

  for(int j=0; j<16; j++)
    ROS_INFO("InverseDynamics coriolis param %d, %f", j, coriolis_param[j]);

  for(int j=0; j<16; j++){
    ROS_INFO("InverseDynamics mass row %d, %f, %f, %f, %f", j,
      inertia_param[j][0], inertia_param[j][1], inertia_param[j][2], inertia_param[j][3]);
  }

}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "inverse_dynamics_test");
  ROS_INFO("InverseDynamics test node");

  ros::NodeHandle nh;

  allegro_kdl_config.parseKdl(nh);
  id_solver = new allegro_hand_kdl::InverseDynamics(allegro_kdl_config);

  ros::Subscriber sub = nh.subscribe<sensor_msgs::JointState>( "joint_states", 3, jointStateCallback);

  ros::spin();

  delete id_solver;
  
  return 0;

}
