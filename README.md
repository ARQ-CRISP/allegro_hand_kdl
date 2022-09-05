# Allego Hand KDL
_A ROS package for controlling Allegro Hand using KDL library_

This package contains wrappers for KDL solvers to control Allegro Hand robot (v3) in different modes. It is tested for ROS-Kinetic distro. It contains interfaces for forward/inverse kinematics and inverse dynamics solvers to control joint and Cartesian position and apply Cartheisan forces. There are also convenience classes for trajectory control.


## Installation

Make sure that Orocos KDL and Eigen3 libraries are installed. In Ubuntu:

```bash
apt install libeigen3-dev
apt install ros-kinetic-orocos-kdl
```

Install [*allegro-hand-ros*](https://github.com/gokhansolak/allegro-hand-ros) according to its README.

Clone this repository and its dependencies under catkin workspace:

```bash
cd <your-catkin-ws>/src
git clone https://github.com/ARQ-CRISP/allegro_hand_kdl.git
git clone https://github.com/ARQ-CRISP/kdl_control_tools.git
git clone https://github.com/ARQ-CRISP/ros_topic_connector.git
```

Run rosdep to install any other dependencies:

```bash
cd <your-catkin-ws>
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

Build the catkin workspace.

## Nodes

Each node has a distinct responsibility. Some of these nodes publish control torques that should be published to Allegro's *torque_cmd* topic. You can redirect the output torques by setting the _TORQUE\_TOPIC_ argument or remapping the nodes. However, usually it is necessary to combine the outputs of multiple controllers, e.g. gravity compensation and trajectory control together. In this case, you can use the ```combine_torques.launch``` file, which already has a mapping for these topics:

```bash
roslaunch allegro_hand_kdl combine_torques.launch
```

#### Gravity compensation node

Gravity compensation node will publish the necessary torques to cancel gravity effect. It depends on the current joint-state and the wrist pose, hence subscribes to topics of these info. It can be run with:

```bash
roslaunch allegro_hand_kdl gravity_compensate.launch TORQUE_TOPIC:=/allegroHand_0/torque_cmd
```

[_TORQUE\_TOPIC_ parameter is optional.](#nodes)

#### Force control node

A force control node works like converter from Cartesian forces to joint torques. It listens to desired Cartesian forces in  */cartesian_forces* topic and publishes the joint torques to achieve these forces at Allegro finger tips to */allegro_torque* topic. Run using:

```bash
roslaunch allegro_hand_kdl control_force.launch TORQUE_TOPIC:=/allegroHand_0/torque_cmd
```

[_TORQUE\_TOPIC_ parameter is optional.](#nodes)

#### Forward kinematics node

#### Joint pose server

Joint pose server can be used to move the robot to a predefined joint pose or maintain the current pose of the robot. It reads the poses from ROS params under *allegro_kdl/poses*. It can be run using:

```bash
roslaunch allegro_hand_kdl joint_pose_control.launch MAINTAIN:=true POSE:=relax
```

If *MAINTAIN* is true, the robot will continuously maintain the latest pose. If no pose is given, it will maintain the initial pose. *POSE* param is optional, and determines the name of the initial pose. See the other parameters in the launch file.

[_TORQUE\_TOPIC_ parameter is optional.](#nodes)

It is possible to change the current pose during runtime by calling the ```/desired_pose``` service with a *PoseRequest*. A PoseRequest should contain the name of a valid pose in *allegro_kdl/poses* params, so that the joint values can be retrieved. It is possible to add new pose entries using [ROS parameter server](http://wiki.ros.org/Parameter%20Server).

The [PoseControlClient](#pose-control-client) class provides a clean interface to interact with the server.

#### Cartesian pose server

This node works just like _Joint pose server_ as explained above, but it takes the poses and controls in the Cartesian space. To run:

```bash
roslaunch allegro_hand_kdl cartesian_pose_control.launch MAINTAIN:=true POSE:=relax
```

It is possible to change the current pose during runtime by calling the ```/desired_cartesian_pose``` service with a *PoseRequest*. List of possible poses are stored in parameter server in the namespace *allegro_kdl/poses/cartesian_poses*. It is possible to add new pose entries using [ROS parameter server](http://wiki.ros.org/Parameter%20Server).

The [PoseControlClient](#pose-control-client) class provides a clean interface to interact with the server.

#### Pose action server

Unlike the two nodes above, this one uses actionlib instead of services; so it publishes feedbacks and the finishing signal. Also this one does not use the ROS parameter server, but takes the finger poses as an input. MAINTAIN and initial POSE parameters are removed.

The following will start the action server, that accepts PoseControl.action requests and publish the torques message (sensor_msgs/JointState):

```bash
roslaunch allegro_hand_kdl pose_action_server.launch TORQUE_TOPIC:=position_torque
```

[_TORQUE\_TOPIC_ parameter is optional.](#nodes)

This action server can be used for both Joint and Cartesian control. To specify the control mode, give only one of the _PoseControl/cartesian\_pose_ and _PoseControl/joint\_pose_ parameters.
An example action client code is given as [a python node](nodes/pose_action_client).

#### Envelop force node

This node applies constant forces towards the centroid of the fingers. It is useful for grasping tasks. To run:

```bash
roslaunch allegro_hand_kdl envelop_force.launch INTENSITY:=1.2 TORQUE_TOPIC:=/allegroHand_0/torque_cmd
```

INTENSITY parameter determines the strength of enveloping forces. It can also be updated during runtime by publishing to */envelop_intensity* topic.

[_TORQUE\_TOPIC_ parameter is optional.](#nodes)

## Classes

#### Pose control client

The [JointPoseClient](include/allegro_hand_kdl/pose_control_client.h) (and [CartesianPoseClient](include/allegro_hand_kdl/pose_control_client.h)) class provides a clean way to interact with the [joint pose server](#joint-pose-server) (and [cartesian pose server](#cartesian-pose-server)). Using an instance of this class, you can move to a pose target or add a new pose to the database without hassle:

```c++
JointPoseClient joint_client = JointPoseClient(nh);
...
joint_client->setTargetName("relax");
joint_client->move();
...
joint_client->setTargetPose(new_pose, "new_name");
joint_client->move();
```

See [pose_client](src/nodes/pose_client.cpp) node for a detailed usage.

### KDL Wrappers

#### Robot description

*AllegroKdlConfig* class is used to create KDL objects that are used for kinematics and inverse dynamics.

#### Kinematics

*Kinematics* class provides methods to calculate the forward and inverse kinematics using KDL library.

#### Inverse dynamics

*InverseDynamics* class is used to derive the dynamic parameters of the hand; coriolis and inertia matrices, gravity vector.

### Controllers

#### Joint-space control

Joint-space control can be achieved using *JointPositionController* class. It has methods for PID control.

#### Cartesian pose control

#### Cartesian force control

#### Trajectory control
