# acin_robot_concol
===

This package contains several control algorithms implemented for an general robot (number of degrees of freedom). The dynamic description must be derived with the Lagrangian Formulation. The dynamic model of the robot is than given by the mass matrix, the coriolis matrix, the gravitation general forces and external forces. Therefore in the package `acin_robot_support` the respective functions for the robot behaviour must be defined. A linear algebra package [Eigen3](http://eigen.tuxfamily.org/index.php?title=Main_Page) is used to perform the calculations within this module.
A B-Spline interpolation is used for the trajectory dispatching received by the controller. The degree of the local b-spline curve as well as the number of via points of the trajectory, which should be interpolated, must be predefined before the compilation. The setting must be specified in the 'CMakeLists.txt'-File of the acin_robot_support package. The same interpolation algorithm is used for the JointSpace trajectory's as well as for the taskspace trajectory's. The taskspace trajectory is specified via the cartesian coordinates (x,y,z) and the orientation can either be specified per euler angles ( e.g. intrinsic rotation z Yaw,  y Pitch, x Roll or extrinsic rotation XYZ) or quaternions. The interpolation of the quaternion is also based on B-Spline algorithm. However the quaternion curve is only C^2 continous.[1]
[1]: http://citeseerx.ist.psu.edu/viewdoc/summary?doi=10.1.1.39.7335
Currently implemented controllers:
* Joint Space Computed Torque
* Cartesian Space Computed Torque
* Singular Perturbation
* Friction Compensation

## Installation

Clone into your catkin workspace:
```
cd YOUR_CATKIN_WS

git clone git@git.acin.tuwien.ac.at:robotics/acin_robot_support.git
git clone git@git.acin.tuwien.ac.at:robotics/acin_robot_control.git
add an additional robot model to the "acin_robot_support\include\acin_robot_support\models" folder
! change the cmake variables "CMAKE_ROBOT_MODEL_PATH" to the proper location of your model description and "CAMKE_ROBOT_DIM" to the number of DoF of your robot.
! The variables can be found in "acin_robot_support\CMakeLists.txt"
```
Remember to recompile your workspace and source `setup.bash` afterwards.

## Usage
``
roslaunch 
``
* ##### Paramter
  * ``
## ROS Topic's
  * ### Cartesian Interface
    List of RoS Topics used be the Controller which is listing on the Cartesian Trajectory Interface:
    * #### Publish
      * `/iiwa_plugin/iiwa_joint_effort (Message Type: std_msgs\Float32MultiArray)`\
        Controller generated effort, send to Gazebo for the Simulation purposes
      * `/spline_generated_trajectory (Message Type: acin_iiwa_control\PoseTraj)`\
        desired interpolated trajectory execute be the control
      * `/task_traj_actual (Message Type: acin_iiwa_control\PoseTraj)`\
        actual task space position of the robot
    * #### Subscribe
      * `/iiwa_plugin/iiwa_joint_state (Message Type: sensor_msgs\JointState)`\
        actual joint states of the robot, e.g. published by the Gazebo simulation
      * `/TaskSpace_trajectory (Message Type: acin_iiwa_control::TaskSpaceTraj)`\
        desired trajectory interface, Cartesian trajectory which consist of 30 points can be send to the controller via this topic

## Messages defiend in the acin_robot_support package
* #### Cartesian Interface Message
  * `TaskSpaceTraj.msg` description:
    ```
    Header header #Header Data of a msg. -- defined in ROS 'std_msgs'
    Pose[] PoseArray #Pose Data  Array -- [0]: pose trajectory point at time[0], [1]...
    time[] #time stamps of the trajectory
    ```
  * `Pose.msg` description:
    ```
    float64[] position #desired position ([0] x; [1] y; [2] z)
    float64[] orientation #desired orientation ([0] Yaw; [1] Pitch; [2] Roll) or Quaternion ([0] w; [1] x; [2] y; [3]z)
    ```
  * `PoseTraj.msg` description:
    ```
    Header header
    geometry_msgs/Pose[] poses# Cartesian Point Pose.position (x,y,z) + Quaternion rientation Pose.orientation Quaternion (w,x,y,z)
    geometry_msgs/Point[] p_vel # Cartesian Point velocity
    geometry_msgs/Point[] omega # angular velocity
    geometry_msgs/Point[] p_accl # Cartesian Point accelerations
    geometry_msgs/Point[] omegaDot # angular accelerations
    ```
