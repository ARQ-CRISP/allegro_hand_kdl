
#include <signal.h>

#include<allegro_hand_kdl/inverse_dynamics.h>

#include<kdl_control_tools/progress_logger.h>

#include <sensor_msgs/JointState.h>

using namespace std;
using namespace allegro_hand_kdl;

AllegroKdlConfig allegro_kdl_config_;
InverseDynamics* id_solver_;

ros::Publisher torque_publisher_; // to control the robot

// timing
ros::Time tstart_;
ros::Time tupdate_;
double duration_ = 6.0;

// PD parameters
vector<double> k_pos_vec_;
vector<double> k_vel_vec_;

// state memory
vector<double> q_initial_;
vector<double> q_last_;
vector<double> qd_last_;

// goal state
int joint_no_ = 6;
double target_state_ = 1.0;

// debug
kdl_control_tools::ProgressLogger p_logger_;

bool state_read_ = false;
bool finished_ = false;

// This procedure is called when the ros node is interrupted
void sigintCallback(int sig)
{
  // save the error trajectory
  string filename_suffix =
      "_"+to_string((int)(k_pos_vec_[joint_no_]*1000))+"_"+
      to_string((int)(k_vel_vec_[joint_no_]*1000))+"_j"+
      to_string(joint_no_)+".csv";
  // p_logger_.writeVarToFile("sj_error", filename_suffix);

  // p_logger_.writeVarToFile("sj_t_grav"+to_string(joint_no_), filename_suffix);
  // p_logger_.writeVarToFile("sj_t_inertia"+to_string(joint_no_), filename_suffix);
  p_logger_.writeVarToFile("sj_e_pos"+to_string(joint_no_), filename_suffix);
  p_logger_.writeVarToFile("sj_e_vel"+to_string(joint_no_), filename_suffix);
  p_logger_.writeVarToFile("sj_tau"+to_string(joint_no_), filename_suffix);

  // send and empty torque message
  sensor_msgs::JointState msg;

  msg.header.stamp = ros::Time::now();

  msg.position.resize(16);
  msg.velocity.resize(16);
  msg.effort.resize(16);

  for (int j=0; j < 16; j++){
    msg.name.push_back("joint_"+to_string(j));
  }

  torque_publisher_.publish(msg);

  // set finished state
  finished_ = true;

  // spin once to hand communication
  ros::spinOnce();

  // All the default sigint handler does is to call shutdown()
  ros::shutdown();
}

void publishTorque(const vector<double>& torque_vec){
  // create a joint state message and publish torques
  sensor_msgs::JointState msg;

  msg.header.stamp = ros::Time::now();

  msg.position.resize(16);
  msg.velocity.resize(16);

  for (int j=0; j < 16; j++){

    // joint torque is sum of the user input and compensation torques.
    double joint_torque = torque_vec[j];

    // emergency stop
    if(joint_torque > 1.2 || joint_torque < -1.2){
      ROS_INFO("Too much torque!");
      sigintCallback(0);
      return;
    }

    // add torques to message as effort
    msg.effort.push_back(joint_torque);
    // names of joints
    msg.name.push_back("joint_"+to_string(j));
  }

  torque_publisher_.publish(msg);
}

void computeTorques(const vector<double>& q, const vector<double>& qd,
                const vector<double>& qdd, vector<double>& tau )
{
  // make sure that tau vector is empty
  tau.clear();

  // in debug mode clean error value to overwrite
  double error_val = 0;

  // Repeat for each finger
  for(int fi=0; fi < FINGER_COUNT; fi++){

    // Create KDL data types
      // inputs
    JntArray q_kdl(FINGER_LENGTH);
    JntArray qd_kdl(FINGER_LENGTH);
    JntArray qdd_kdl(FINGER_LENGTH);
      // trajectory values
    Eigen::VectorXd q_des_eigen(FINGER_LENGTH);
    Eigen::VectorXd qd_des_eigen(FINGER_LENGTH);
    Eigen::VectorXd qdd_des_eigen(FINGER_LENGTH);
      // ID parameters
    JntArray gravity_kdl(FINGER_LENGTH);
    // JntArray coriolis_kdl(FINGER_LENGTH);
    JntSpaceInertiaMatrix inertia_kdl(FINGER_LENGTH);
      // output
    JntArray tau_kdl(FINGER_LENGTH);

    // Copy inputs from arguments
    for(int si=0; si<FINGER_LENGTH; si++){ // si = segment index
      q_kdl(si) = q[fi * FINGER_LENGTH + si];
      qd_kdl(si) = qd[fi * FINGER_LENGTH + si];
      qdd_kdl(si) = qdd[fi * FINGER_LENGTH + si];
    }

    // copt current to desired
    for(int si=0; si<FINGER_LENGTH; si++){ // si = segment index
      q_des_eigen(si) = q_initial_[(fi * FINGER_LENGTH) + si];
      qd_des_eigen(si) = 0.0;
    }
    // set eigen vector for desired state
    if(joint_no_ >= fi * FINGER_LENGTH && joint_no_ < (fi+1) * FINGER_LENGTH)
      q_des_eigen(joint_no_ % FINGER_LENGTH) = target_state_;

    // Get the inverse dynamics parameters for current configuration/motion
    id_solver_->getGravityParam_kdl(fi, q_kdl, gravity_kdl);
    // id_solver_->getCoriolisParam(fi, q_kdl, qd_kdl, coriolis_kdl);
    id_solver_->getMassParam_kdl(fi, q_kdl, inertia_kdl);

    // convert gain vectors to kdl type
    JntArray k_pos_kdl(FINGER_LENGTH), k_vel_kdl(FINGER_LENGTH);
    for(int si=0; si<FINGER_LENGTH; si++){
      k_pos_kdl(si) = k_pos_vec_[fi * FINGER_LENGTH + si];
      k_vel_kdl(si) = k_vel_vec_[fi * FINGER_LENGTH + si];
    }

    // Calculate the torque using inverse dynamics motion equation
    // using Eigen data types
    Eigen::VectorXd g = gravity_kdl.data * 4.2;
    Eigen::MatrixXd m = inertia_kdl.data;
    Eigen::VectorXd e_pos = k_pos_kdl.data.cwiseProduct(q_des_eigen - q_kdl.data);
    Eigen::VectorXd e_vel = k_vel_kdl.data.cwiseProduct(qd_des_eigen - qd_kdl.data);

    Eigen::VectorXd t_ =
      (e_pos+e_vel);

    // Add output to std vector
    for(int si=0; si<FINGER_LENGTH; si++){ // si = segment index
      tau.push_back(t_(si));

      // log the torque components
      p_logger_.log("sj_t_grav"+to_string(fi*FINGER_LENGTH+si), g(si));
      p_logger_.log("sj_t_inertia"+to_string(fi*FINGER_LENGTH+si), m(si));
      p_logger_.log("sj_e_pos"+to_string(fi*FINGER_LENGTH+si), e_pos(si));
      p_logger_.log("sj_e_vel"+to_string(fi*FINGER_LENGTH+si), e_vel(si));
      p_logger_.log("sj_tau"+to_string(fi*FINGER_LENGTH+si), t_(si));
    }

    // add error of each joint
    for(int si=0; si<FINGER_LENGTH; si++){ // si = segment index
      // difference of q_des and q_actual is error
      error_val += q_des_eigen(si) - q[(fi * FINGER_LENGTH) + si];
    }

  }

  p_logger_.log("sj_error", error_val);
}



void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg) {

  // avoid working after finishing
  if(finished_) return;

  // *** initial state
  if(!state_read_){
    ROS_INFO("SingleJointControl starting control");

    // start the timer
  	tstart_ = ros::Time::now();
    tupdate_ = ros::Time::now();

    // init state memory
    q_initial_ = msg->position;
    q_last_ = q_initial_;
    qd_last_.resize(msg->position.size()); // zeros

    state_read_ = true;
    return;
  }

  // *** during cont

  // Calc time since start
  ros::Time tnow_ = ros::Time::now();
  double dt_start = (tnow_ - tstart_).sec + 1e-9 * (tnow_ - tstart_).nsec;
  double dt_update = (tnow_ - tupdate_).sec + 1e-9 * (tnow_ - tupdate_).nsec;

  // // check minimum time step
  // if( dt_update < 0.01){
  //   // wait more
  //   return;
  // }

  // Termination time?
  if (dt_start > duration_){
    ROS_INFO("SingleJointControl finished");
    sigintCallback(SIGINT);
    return;
  }

  // copy state
  vector<double> q = msg->position;
  // calc derivatives
  vector<double> qd(q.size());
  vector<double> qdd(q.size());

  for (unsigned int di=0; di < q.size(); di++){
    // calc derivative
    qd[di] = (q[di] - q_last_[di]) / dt_update;
    qdd[di] = (qd[di] - qd_last_[di]) / dt_update;
  }

  // calculate torques to follow trajectory
  vector<double> torques;

  computeTorques(q, qd, qdd, torques);
  
  publishTorque(torques);

  // update state memory
  q_last_ = q;
  qd_last_ = qd;

  // update timing
  tupdate_ = ros::Time::now();

}

double parseDouble(double def_val){
  string in_str;
  getline(cin, in_str);

  // nothing entered
  if(in_str.length() == 0)
    return def_val;

  try{
    // something entered, greater than zero
    return max(0.0, stod(in_str));
  }
  catch (const std::invalid_argument& ia) {
	  std::cerr << "Invalid argument: " << ia.what() << '\n';
  }

  // return default if invalid argument
  return def_val;
}

void promptParams(){

  cout << "\njoint_no (def. " << joint_no_ <<"): ";
  joint_no_ = (int) parseDouble(joint_no_);

  cout << "\ntarget_state (def. " << target_state_ <<"): ";
  target_state_ = parseDouble(target_state_);

  double k_pos;
  cout << "\nk_pos (def.  0.50): ";
  k_pos_vec_[joint_no_] = parseDouble(0.50);

  double k_vel;
  cout << "\nk_vel (def. 0.04): ";
  k_vel_vec_[joint_no_] = parseDouble(0.04);

  cout << "\nduration (def. " << duration_ <<"): ";
  duration_ = parseDouble(duration_);

}

bool getControlGains(ros::NodeHandle& nh, vector<double>& k_pos_vec, vector<double>& k_vel_vec){

  k_pos_vec.resize(FINGER_LENGTH*FINGER_COUNT);
  k_vel_vec.resize(FINGER_LENGTH*FINGER_COUNT);

  string param_name;
  if(!nh.searchParam("/allegroHand_right_0/gains/pose", param_name) ){
    ROS_ERROR("SingleJointControl: Can't find joint_limits param.");
    return false;
  }

  for(int fi=0; fi < FINGER_COUNT; fi++)
    for(int si=0; si < FINGER_LENGTH; si++){
      ros::param::get(param_name+"/p/j"+to_string(fi)+to_string(si),
        k_pos_vec[fi*FINGER_LENGTH+si]);
      ros::param::get(param_name+"/d/j"+to_string(fi)+to_string(si),
        k_vel_vec[fi*FINGER_LENGTH+si]);
    }

  return true;
}


int main(int argc, char **argv){

  ros::init(argc, argv, "single_joint_control_node");
  ROS_INFO("SingleJointControl test node");

  ros::NodeHandle nh;

  // create dynamics model
  allegro_kdl_config_.parseKdl(nh);
  id_solver_ = new InverseDynamics(allegro_kdl_config_);

  // get the pd gain params if exists
  if(!getControlGains(nh, k_pos_vec_, k_vel_vec_)){
    // otherwise use default for all
    k_pos_vec_.resize(FINGER_LENGTH*FINGER_COUNT, 400.0);
    k_vel_vec_.resize(FINGER_LENGTH*FINGER_COUNT, 30.0);
  }

  promptParams();

  // ros shutdown handler
  signal(SIGINT, sigintCallback);

  // countdown
  ros::Duration sleep_time(1.0);

  for (int ci = 3; ci > 0; ci--){
    ROS_INFO( "SingleJointControl: starting in %d",ci );
    sleep_time.sleep();
  }

  ROS_INFO( "SingleJointControl: started!" );

  // create ros communication nodes
  torque_publisher_ =
    nh.advertise<sensor_msgs::JointState>("torque", 1);
  ros::Subscriber sub =
    nh.subscribe<sensor_msgs::JointState>( "joint_states", 1, jointStateCallback, ros::TransportHints().tcpNoDelay().reliable());



  ros::spin();

  delete id_solver_;

  return 0;

}
