#ifndef DYNAMIC_PLANNER_H
#define DYNAMIC_PLANNER_H

// IMPORT LIBRARIES

//C++ Libraries  
#include <unordered_map> // https://en.cppreference.com/w/cpp/container/unordered_map

// ROS Libaries
#include <control_msgs/JointTrajectoryControllerState.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// MoveIt! Libaries
#include "moveit/trajectory_processing/iterative_spline_parameterization.h"
#include <moveit/collision_detection/collision_tools.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

// Global constant definition of number of joints 
#define JOINTS_NUM 6

// Struct definition of the parameters of the Dynamic Planner
struct DynamicPlannerParams
{
  std::string name     = "RRTConnect";        // look up from list in ompl_planning.yaml
  int num_attempts     = 5;                   // max number of attempts to find a trajectory
  double planning_time = 0;                   // maximum planning time in seconds (default 0: no limit)
  double vel_factor    = 99.;                 // velocity factor
  double acc_factor    = 99.;                 // acceleration factor
  moveit_msgs::Constraints path_constraints;  // moveit vector of path constraints

  // Struct constructor declaration for the dynamic planner params setup
  DynamicPlannerParams() {}
  DynamicPlannerParams(const std::string& planner_id, const int attempts,
                       const double time, const double v_factor, const double a_factor)
    : name(planner_id), 
      num_attempts(attempts), 
      planning_time(time), 
      vel_factor(v_factor),
      acc_factor(a_factor)
  {}
};

// Class dynamic planner
class DynamicPlanner
{
  // Public constructor declaration
 public:
  // three different constructor declaration with different initialized params values
  // FIRST POSSIBLE DECLARATION -> short form
  DynamicPlanner(const std::string& manipulator_name,
                 const std::vector<std::string>& joints_name, 
                 const double v_factor = 0.2,
                 const double a_factor = 0.2);
  // SECOND POSSIBLE DECLARATION -> two planning groups
  DynamicPlanner(const std::string& planning_group1_name,
                 const std::string& planning_group2_name,
                 const std::vector<std::string>& joints_names_group1,
                 const std::vector<std::string>& joints_names_group2,
                 const double v_factor = 0.2, 
                 const double a_factor = 0.2);
  // THIRD POSSIBLE DECLARATION -> three planning groups
  DynamicPlanner(const std::string& planning_group1_name,
                 const std::string& planning_group2_name, 
                 const std::string& gripper_name,
                 const std::vector<std::string>& joints_names_group1,
                 const std::vector<std::string>& joints_names_group2,
                 const std::vector<std::string>& joints_names_gripper,
                 const double v_factor = 0.2, 
                 const double a_factor = 0.2);

  // PUBLIC VARIABLES DECLARATIONS made by Italo
  moveit_msgs::RobotTrajectory  trajectory_;
  ulong                         trajpoint_;
  std::vector<moveit_msgs::CollisionObject>           collision_objects_;
  std::vector<moveit_msgs::AttachedCollisionObject>   attached_objects_;
  planning_scene::PlanningScenePtr                    planning_scene_ = nullptr;
  moveit_visual_tools::MoveItVisualToolsPtr           visual_tools_   = nullptr;
  moveit::planning_interface::PlanningSceneInterface  planning_scene_interface_;

  // PUBLIC FUNCTIONS DEFINITIONS
  // MoveIt! gets planning scene and collision objects
  std::vector<moveit_msgs::CollisionObject>& getCollisionObjects();
  std::vector<moveit_msgs::AttachedCollisionObject>& getAttachedCollisionObjects();
  moveit::planning_interface::PlanningSceneInterface& getPlanningSceneInterface();
  const planning_scene::PlanningScenePtr getPlanningScenePtr();
  const moveit_visual_tools::MoveItVisualToolsPtr getVisualToolsPtr();

  // Dynamic planner parameters getter and setter
  DynamicPlannerParams getParams() const { return params_; }  // Getter is already implemented here!!!
  // The 'const' indicates that this member function does not modify the state of the object it is called on.
  void setParams(const std::string& planner_id, const int attempts, const double time,
                 const double v_factor, const double a_factor);
  void setParams(const DynamicPlannerParams& params);

  // Enable and disable of the Elevation check for the gripper with a given radians angle value
  void activateElevationCheck(const std::string ee_link,
                              const double min_elevation_angle);
  void deactivateElevationCheck();

  // Set joints limits as vectors of min-max angles
  void setJointsLimits(const std::vector<std::string>&  joints_names,
                       const std::vector<double>&       max_angles,
                       const std::vector<double>&       min_angles);

  // Path and wrist constraints setup 
  void setWristGoalConstraints(const bool value);
  void setPathConstraint(const moveit_msgs::Constraints constraints);
  void clearPathConstraint();
  
  // Robot collision safe zones setup
  void setPadding(const double link_padding); // enlarge the size of robot model contours 
  void setScale(const double link_scale);     // scale the collision by a primitive size

  // Simulation setup -> true for sim or false for debug
  void setSimMode(const bool value);

  // PLANNING FUNCTIONS -> each versions is adapt for different kind of input types
  // Single JOINT goal
  void plan(const std::vector<double>& final_position);
  void plan(const std::vector<double>& final_position,
            const std::string& joint_model_group_name);
  void plan(const std::vector<double>& final_position,
            const std::string& joint_model_group_name,
            const robot_state::RobotState& robot_state);
  // Multiple JOITNS goals
  void plan(const std::vector<std::vector<double>>& positions);
  void plan(const std::vector<std::vector<double>>& positions,
            const std::string& joint_model_group_name);
  // Single POSITION goal (within 3D carthesian space, operative space)
  void plan(const geometry_msgs::PoseStamped& final_pose, const std::string& link_name);
  void plan(const geometry_msgs::PoseStamped& final_pose, const std::string& link_name,
            const std::string& joint_model_group_name);
  void plan(const geometry_msgs::PoseStamped& final_pose, const std::string& link_name,
            const std::string& joint_model_group_name,
            const robot_state::RobotState& robot_state);

  // Check trajectory feasibility as long as the dynamic object moves
  void checkTrajectory();

  // MOVE ROBOT FUNCTIONS
  // Robot moving function given joint states 
  void moveRobot(const sensor_msgs::JointState& joint_states);
  // Robot moving function given a trajectory setpoint
  void moveRobot(const moveit_msgs::RobotTrajectory& robot_trajectory);

  // ROS spinning function
  void spinner(void);

  // Check if dynamic planner is ready to work
  bool isReady() const;

  // PRIVATE VARIABLES AND FUNCTIONS DECLARATIONS
 private:

  // Planning space enum definition -> standard joints and cartesian spaces
  enum PlanningSpace : bool
  {
    JOINTS_SPACE,     // 0 if we didn't want to use the enum
    CARTESIAN_SPACE   // 1 if we didn't want to use the enum
  };

  // ROS node, publishers and subscribers definition
  ros::NodeHandle nh_;            // node to use as ROS base file for pubs and subs
  ros::Publisher  joints_pub_;    // publisher to moveit fake joints controllers
  ros::Publisher  trajectory_pub_;// trajectory publisher to /trajectory topic
  ros::Publisher  stop_pub_;      // publisher to stop trajectory execution 
  ros::Subscriber joints_sub_;    // subscriber to joint states
  ros::Subscriber trajpoint_sub_; // subscriber to the trajectory counter

  // Dynamic planner params definition
  DynamicPlannerParams params_;

  // Planning and gripper groups
  std::string planning_group_name1_;
  std::string planning_group_name2_;
  std::string gripper_name_;
  // Joints and gripper group names
  std::vector<std::string> joints_names_group1_;
  std::vector<std::string> joints_names_group2_;
  std::vector<std::string> joints_names_gripper_;
  // Joints and grippers values
  std::vector<double> joints_values_group1_;
  std::vector<double> joints_values_group2_;
  std::vector<double> joints_values_gripper_;
  // Joints and gripper maps to store joints/gripper values
  std::unordered_map<std::string, double> joints_map_group1_; // Set group2 jnts names through mapping
  std::unordered_map<std::string, double> joints_map_group2_; // Set group1 jnts names through mapping
  std::unordered_map<std::string, double> joints_map_gripper_;// Set jnts+EE names through mapping

  // Set joints limit through mapping the name of the configuration with a user defined 
  // list of min-max values on pos-vel-acc
  // While the desired limits are forced by user within the robot model for planning purposes, 
  // the following map structure holds the old values, so limits set with model loading are stored 
  std::unordered_map<std::string, moveit::core::VariableBounds> original_joints_limits_map_;

  // ROBOT MODEL AND STATE DEFINITION
  
  // Basic state and model import variables
  robot_model_loader::RobotModelLoader  robot_model_loader_;
  robot_model::RobotModelPtr            robot_model_ = nullptr;
  robot_state::RobotStatePtr            robot_state_ = nullptr;
  // Joint and grupper model groups
  const robot_model::JointModelGroup*   joint_model_group1_;
  const robot_model::JointModelGroup*   joint_model_group2_;
  const robot_model::JointModelGroup*   joint_model_gripper_;

  // PLANNING SCENE SETUP

  // Collision objects variables
  // std::vector<moveit_msgs::CollisionObject>         collision_objects_;
  // std::vector<moveit_msgs::AttachedCollisionObject> attached_objects_;
  // Visual and interface tools setup
  // planning_scene::PlanningScenePtr          planning_scene_ = nullptr;
  // moveit_visual_tools::MoveItVisualToolsPtr visual_tools_   = nullptr; MADE PUBLIC
  // moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  // Planning pipeline client setup
  planning_pipeline::PlanningPipelinePtr planning_pipeline_ = nullptr;
  planning_interface::MotionPlanRequest  request_;// Client request  to MoveIt
  planning_interface::MotionPlanResponse result_; // Client result from MoveIt

  // Planning names
  PlanningSpace planning_space_;    // MoveIt! planning space object, parent of useful functions
  std::string planning_link_name_;  // the name of the link as parent frame of the goal pose
  std::string planning_group_;      // name of the group of joints to be controlled

  // Planning outputs
  moveit_msgs::Constraints      goal_;          // Moevit msgs about goal constraints 
  // moveit_msgs::RobotTrajectory  trajectory_;    // Moevit msgs about trajectory MADE PUBLIC
  std::vector<double>           final_position_;// Final joints position
  geometry_msgs::PoseStamped    final_pose_;    // Final TCP 3D position

  // ulong trajpoint_;               // Current point of the trajectory considered MADE PUBLIC
  ulong invalid_state_;           // point of the trajectory where a collision has been found
  bool success_;                  // whereas trajectory planning has been successfull
  bool joints_group1_received_;   // Check if joints group 1 has been received from the planner
  bool joints_group2_received_;   // Check if joints group 2 has been received from the planner
  bool joints_gripper_received_;  // Check if joints gripper has been received from the planner
  bool reversed_joints_;          // INCLUDED INTO EACH CONSTRUCTOR, BUT IT'NOT USED INTO THE CODE ???
  bool obstruction_;              // wheather an obstacle is on the robot path (1) or not (0)
  bool sim_;                      // Simulation status: 1 for simulation, 0 for debug
  bool wrist_goal_constraints_;   // Constraints applied to the goal for robot gripper wrist

  // End-effector elevation check
  std::string ee_link_name_;            // Gripper link name
  double min_elevation_angle_;          // end effector elevation angle setup but the user
  bool elevation_angle_check_ = false;  // check if elevation angle outcomes the limit value

  // Initialization of the planner, the node and other params
  void initialize(const double v_factor, const double a_factor);

  // Joints limit handling
  void adaptJointsLimits();           // enlarge joints limits
  void restoreOriginalJointsLimits(); // restore original joints limits as loaded by robot model

  // Merge a user defined trajectory with the running one depending on their relative length
  void merge(moveit_msgs::RobotTrajectory& robot_trajectory);

  // Check if the y-axis rotation of the ee link overcomes an user defined value
  bool checkElevationAngle(const robot_state::RobotStatePtr rs);

  bool checkTrajectoryElevation(const moveit_msgs::RobotTrajectory& robot_trajectory);

  // MoveIt! planning to goal
  void plan(const moveit_msgs::Constraints& desired_goal, const bool send = true);
  // MoveIt! planning to many goals through trajctory
  void plan(const std::vector<moveit_msgs::Constraints>& desired_goals,
            moveit_msgs::RobotTrajectory& robot_trajectory);

  // CALLBACK FUNCTIONS FOR SUBSCRIBERS
  // Trajectory points current state 
  void trajPointCallback(const std_msgs::Int32::ConstPtr& traj_point);
  // Joints current state
  void jointsCallback(const sensor_msgs::JointState::ConstPtr& joints_state);
};

#endif /* DYNAMIC_PLANNER_H */
