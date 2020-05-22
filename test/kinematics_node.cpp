#include <math.h>

#include <allegro_hand_kdl/kinematics.h>

#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/Transform.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>

using namespace std;
using namespace allegro_hand_kdl;

AllegroKdlConfig allegro_kdl_config;
Kinematics* kinematics_solver;

tf2_ros::Buffer tfBuffer;

ros::Publisher fk_pos_error_publisher;
ros::Publisher ik_pos_error_publisher;

vector<double> q_init_;
vector<double> q_current_;

std_msgs::Float32 ik_pos_error_;
std_msgs::Float32 fk_pos_error_;


void getFingertipTransforms(vector<geometry_msgs::Pose>& poses){
  poses.resize(FINGER_COUNT);
  // get the tf transforms of fingers and the thumb
  try{
    // transforms of finger tips w.r.t. hand root
    for (int fi=0; fi<FINGER_COUNT; fi++){
      int tip_index = fi * FINGER_LENGTH + 3;
      // get transform from TF server
      geometry_msgs::Transform transform =
        tfBuffer.lookupTransform(
          "hand_root",
          "link_"+ to_string(tip_index) +"_tip",
          ros::Time(0)).transform;
      // convert to pose
      poses[fi].orientation = transform.rotation;
      poses[fi].position.x = transform.translation.x;
      poses[fi].position.y = transform.translation.y;
      poses[fi].position.z = transform.translation.z;
    }
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
    return;
  }
}

void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg) {

  // update the state
  q_current_ = msg->position;

}

void timerCallback(const ros::TimerEvent&){
  // got the joint states?
  if(q_current_.size() < FINGER_LENGTH*FINGER_COUNT) return;

  // init state as a seed for IK solver, this one is read once and kept the same
  if(q_init_.size()< q_current_.size())
    q_init_ = q_current_;

  // get tf info
  vector<geometry_msgs::Pose> fingertip_tf_vec;
  getFingertipTransforms(fingertip_tf_vec);

  if(fingertip_tf_vec.size() < FINGER_COUNT){
    ROS_WARN("Kinematics: couldn't get fingertip transforms");
    return;
  }

  // test forward kinematics
  vector<geometry_msgs::Pose> fk_result;
  kinematics_solver->calcCartPos(q_current_, fk_result);

    // find error
  fk_pos_error_.data = 0.0;
  for(int fi=0; fi < FINGER_COUNT; fi++){
    fk_pos_error_.data +=
      abs(fk_result[fi].position.x-fingertip_tf_vec[fi].position.x)+
      abs(fk_result[fi].position.y-fingertip_tf_vec[fi].position.y)+
      abs(fk_result[fi].position.z-fingertip_tf_vec[fi].position.z);
  }

  ROS_INFO("Kinematics: FK pos error: %f", fk_pos_error_.data);
  fk_pos_error_publisher.publish(fk_pos_error_);

  // test inverse kinematics position

  vector<double> ik_pos_result;
  int err_code;
  err_code = kinematics_solver->calcJointPos(q_init_, fingertip_tf_vec, ik_pos_result);

  ROS_INFO("Kinematics: IK calculated");
  if(err_code != 0)
    ROS_WARN("Kinematics: error code %d.", err_code);

    // find error
  ik_pos_error_.data = 0.0;
  for(int ji=0; ji < FINGER_COUNT*FINGER_LENGTH; ji++){
    ik_pos_error_.data += abs(q_current_[ji]-ik_pos_result[ji]);
  }

  ROS_INFO("Kinematics: IK pos error: %f", ik_pos_error_.data);
  ik_pos_error_publisher.publish(ik_pos_error_);

  // // test inverse kinematics velocity
  // vector<double> ik_vel_result;
  // kinematics_solver->calcJointVel(q_current_, fingertip_twist_vec, ik_vel_result);
}

bool getJointLimits(ros::NodeHandle& nh, vector<double>& qmin, vector<double>& qmax){

  qmin.resize(FINGER_LENGTH*FINGER_COUNT);
  qmax.resize(FINGER_LENGTH*FINGER_COUNT);

  string param_name;
  if(!nh.searchParam("/allegroHand_right_0/joint_limits", param_name) ){
    ROS_ERROR("Kinematics: Can't find joint_limits param.");
    return false;
  }

  for(int fi=0; fi < FINGER_COUNT; fi++)
    for(int si=0; si < FINGER_LENGTH; si++){
      ros::param::get(param_name+"/min/j"+to_string(fi)+to_string(si),
        qmin[fi*FINGER_LENGTH+si]);
      ros::param::get(param_name+"/max/j"+to_string(fi)+to_string(si),
        qmax[fi*FINGER_LENGTH+si]);
    }

  return true;
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "kinematics_test");
  ROS_INFO("Kinematics test node");

  ros::NodeHandle nh;

  // get the joint limits if exists
  vector<double> qmin;
  vector<double> qmax;
  if(!getJointLimits(nh, qmin, qmax)){
    nh.shutdown();
    return -1;
  }

  // create solver
  allegro_kdl_config.parseKdl(nh);
  kinematics_solver = new Kinematics(allegro_kdl_config, 3);//, qmin, qmax);

  ros::Subscriber sub = nh.subscribe<sensor_msgs::JointState>("joint_states", 3, jointStateCallback);


  // create tf listener
  tf2_ros::TransformListener tfListener(tfBuffer);

  // create publishers
  fk_pos_error_publisher =
    nh.advertise<std_msgs::Float32>("fk_pos_error", 1);
  ik_pos_error_publisher =
    nh.advertise<std_msgs::Float32>("ik_pos_error", 1);

  // start the test loop
  ros::Timer timer = nh.createTimer(ros::Duration(2.0), &timerCallback); // test every 1 second

  ros::spin();

  delete kinematics_solver;

  return 0;

}
