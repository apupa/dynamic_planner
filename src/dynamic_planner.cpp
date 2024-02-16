// CLASS IMPLEMENTATION OF DYNAMIC PLANNER

// IMPORT LIBRARIES
#include "dynamic_planner/dynamic_planner.h"

// 3 versions of public dynamic planner constructors
// V1: 4 args, 12 private variables initializers -> ONLY 1 PLANNING GROUP

// THE CONTENT OF THIS FUNCTION CAN BE REUSED IN ALL THE OTHER 2 FUNCTIONS
/* EX. V2
{
  DynamicPlanner(planning_group1_name, joints_names_group1, v_factor, a_factor)
  DynamicPlanner(planning_group2_name, joints_names_group2, v_factor, a_factor)
}
*/
/* EX. V3
{
  DynamicPlanner(planning_group1_name, joints_names_group1, v_factor, a_factor)
  DynamicPlanner(planning_group2_name, joints_names_group2, v_factor, a_factor)
  DynamicPlanner(gripper_name,         joints_names_gripper,v_factor, a_factor)
}
*/
// CON FUNZIONE COSTRUTTORE SUPPLEMENTARE

DynamicPlanner::DynamicPlanner(const std::string& manipulator_name,
                               const std::vector<std::string>& joints_name,
                               const double v_factor, const double a_factor)
  : nh_("~"), planning_group_name1_(manipulator_name), joints_names_group1_(joints_name),
    trajpoint_(0UL), success_(true), joints_group1_received_(false),
    joints_group2_received_(true), joints_gripper_received_(true), reversed_joints_(true),
    obstruction_(false), sim_(false), wrist_goal_constraints_(false)
{
  // Load ROS pubs/subs, planning scene, robot model and state, visual tools and the
  // planner
  initialize(v_factor, a_factor);

  // Initialize joints map for robot state update: per each joint name, set its value to 0
  for (const std::string& name : joints_names_group1_)
    joints_map_group1_[name] = 0;

  // Adjust size
  joints_values_group1_.resize(joints_names_group1_.size());

  // Wait until we receive joints values of planning group 1
  ros::Rate loop_rate(10);
  // SHOULD THIS BE FROM LAUNCH PARAM 'sample_duration' useful for ubnifporm samplefilter
  // OK I know that this loop rate is used only to check if ros pubs are ready, but within
  // this class spinner() function, no other loop rate is set

  // Continue ro check readiness of ros pubs
  while (!isReady())
  {
    loop_rate.sleep();
    ros::spinOnce();
  }

  // Update robot state
  joint_model_group1_ = robot_state_->getJointModelGroup(planning_group_name1_);
  robot_state_->setJointGroupPositions(
    joint_model_group1_, joints_values_group1_); // UPDATE OFFICIAL JOINTS STATE

  // Move virtual robot to initial position
  sensor_msgs::JointState initial_pose_msg;
  initial_pose_msg.name     = joints_names_group1_;
  initial_pose_msg.position = joints_values_group1_;
  moveRobot(initial_pose_msg);
}

// V2: 4 args, 14 private variables initializers -> 2 PLANNING GROUPS
DynamicPlanner::DynamicPlanner(const std::string& planning_group1_name,
                               const std::string& planning_group2_name,
                               const std::vector<std::string>& joints_names_group1,
                               const std::vector<std::string>& joints_names_group2,
                               const double v_factor /*=0.2*/,
                               const double a_factor /*=0.2*/)
  : nh_("~"), planning_group_name1_(planning_group1_name),
    planning_group_name2_(planning_group2_name),
    joints_names_group1_(joints_names_group1), joints_names_group2_(joints_names_group2),
    trajpoint_(0UL), success_(true), joints_group1_received_(false),
    joints_group2_received_(false), joints_gripper_received_(true),
    reversed_joints_(true), obstruction_(false), sim_(false),
    wrist_goal_constraints_(false)
{
  initialize(v_factor, a_factor);

  // Initialize joints map for robot state update (group 1 and 2)
  for (const std::string& name : joints_names_group1_)
    joints_map_group1_[name] = 0;
  joints_values_group1_.resize(joints_names_group1_.size());
  for (const std::string& name : joints_names_group2_)
    joints_map_group2_[name] = 0;
  joints_values_group2_.resize(joints_names_group2_.size());

  // Wait until we receive joints values of both planning groups
  ros::Rate loop_rate(10);
  while (!isReady())
  {
    loop_rate.sleep();
    ros::spinOnce();
  }
  // Update robot state
  joint_model_group1_ = robot_state_->getJointModelGroup(planning_group_name1_);
  joint_model_group2_ = robot_state_->getJointModelGroup(planning_group_name2_);
  robot_state_->setJointGroupPositions(joint_model_group1_, joints_values_group1_);
  robot_state_->setJointGroupPositions(joint_model_group2_, joints_values_group2_);

  // Move virtual robot to initial position
  sensor_msgs::JointState initial_pose_msg;
  initial_pose_msg.name     = joints_names_group1_;
  initial_pose_msg.position = joints_values_group1_;
  moveRobot(initial_pose_msg);
  initial_pose_msg.name     = joints_names_group2_;
  initial_pose_msg.position = joints_values_group2_;
  moveRobot(initial_pose_msg);
}

// V3: 8 args, 16 private variables initializers -> 3 PLANNING GROUPS
DynamicPlanner::DynamicPlanner(const std::string& planning_group1_name,
                               const std::string& planning_group2_name,
                               const std::string& gripper_name,
                               const std::vector<std::string>& joints_names_group1,
                               const std::vector<std::string>& joints_names_group2,
                               const std::vector<std::string>& joints_names_gripper,
                               const double v_factor /*=0.2*/,
                               const double a_factor /*=0.2*/)
  : nh_("~"), planning_group_name1_(planning_group1_name),
    planning_group_name2_(planning_group2_name), gripper_name_(gripper_name),
    joints_names_group1_(joints_names_group1), joints_names_group2_(joints_names_group2),
    joints_names_gripper_(joints_names_gripper), trajpoint_(0UL), success_(true),
    joints_group1_received_(false), joints_group2_received_(false),
    joints_gripper_received_(false), reversed_joints_(true), obstruction_(false),
    sim_(false), wrist_goal_constraints_(false)
{
  initialize(v_factor, a_factor);

  // Initialize joints map for robot state update
  for (const std::string& name : joints_names_group1_)
    joints_map_group1_[name] = 0;
  joints_values_group1_.resize(joints_names_group1_.size());
  for (const std::string& name : joints_names_group2_)
    joints_map_group2_[name] = 0;
  joints_values_group2_.resize(joints_names_group2_.size());
  for (const std::string& name : joints_names_gripper_)
    joints_map_gripper_[name] = 0;
  joints_values_gripper_.resize(joints_names_gripper_.size());

  // Wait until we receive joints values of both planning groups
  ros::Rate loop_rate(10);
  while (!isReady())
  {
    loop_rate.sleep();
    ros::spinOnce();
  }
  // Update robot state
  joint_model_group1_  = robot_state_->getJointModelGroup(planning_group_name1_);
  joint_model_group2_  = robot_state_->getJointModelGroup(planning_group_name2_);
  joint_model_gripper_ = robot_state_->getJointModelGroup(gripper_name_);
  robot_state_->setJointGroupPositions(joint_model_group1_, joints_values_group1_);
  robot_state_->setJointGroupPositions(joint_model_group2_, joints_values_group2_);
  robot_state_->setJointGroupPositions(joint_model_gripper_, joints_values_gripper_);

  // Move virtual robot to initial position
  sensor_msgs::JointState initial_pose_msg;
  initial_pose_msg.name     = joints_names_group1_;
  initial_pose_msg.position = joints_values_group1_;
  moveRobot(initial_pose_msg);
  initial_pose_msg.name     = joints_names_group2_;
  initial_pose_msg.position = joints_values_group2_;
  moveRobot(initial_pose_msg);
  // WHY JOINT MODEL GRIPPER IS NOT MOVED TO THE INITIAL POSITION POSSIAMO RITORNARE AL
  // SOLO ROBOT OK initial_pose_msg.name     = joints_names_gripper_;
  // initial_pose_msg.position = joints_values_gripper_;
  // moveRobot(initial_pose_msg); NON CE' IL GRIPPE RIN SIM
}

//---------- PUBLIC FUNCTIONS ------------------------------------------------//

// Getter functions (they return a global library variable value)
std::vector<moveit_msgs::CollisionObject>& DynamicPlanner::getCollisionObjects()
{
  return collision_objects_;
}

std::vector<moveit_msgs::AttachedCollisionObject>&
DynamicPlanner::getAttachedCollisionObjects()
{
  return attached_objects_;
}

moveit::planning_interface::PlanningSceneInterface&
DynamicPlanner::getPlanningSceneInterface()
{
  return planning_scene_interface_;
}

const planning_scene::PlanningScenePtr DynamicPlanner::getPlanningScenePtr()
{
  return planning_scene_;
}

const moveit_visual_tools::MoveItVisualToolsPtr DynamicPlanner::getVisualToolsPtr()
{
  return visual_tools_;
}

// Setter functions for the Dynamic Planner
void DynamicPlanner::setParams(const std::string& planner_id, const int attempts,
                               const double time, const double v_factor,
                               const double a_factor)
{
  // Replace planner parameters within the struct BUT PASSED AS SEPARATE VARIABLES
  params_ = DynamicPlannerParams(planner_id, attempts, time, v_factor, a_factor);
}

void DynamicPlanner::setParams(const DynamicPlannerParams& params)
{
  // Replace planner parameters within the struct PASSED BY A STRUCT
  params_ = params;
}

// Activation of the elevation check for the EE_link (its implementation
// is in the private funcion checkElevationAngle() )
void DynamicPlanner::activateElevationCheck(const std::string ee_link,
                                            const double min_elevation_angle)
{
  ee_link_name_          = ee_link;
  min_elevation_angle_   = min_elevation_angle; // Store the desired value in radians
  elevation_angle_check_ = true;                // Bool enabling
  // Log and deg conversion for user
  ROS_INFO("Elevation angle check activated: minimum angle = %.0f [deg]",
           min_elevation_angle * 180 / M_PI);
}

void DynamicPlanner::deactivateElevationCheck()
{
  // Empty elevation check variables
  ee_link_name_          = "";
  min_elevation_angle_   = 1.0;
  elevation_angle_check_ = false;
  ROS_INFO("Elevation angle check deactivated");
}

// Set Joints Limits for trajectory execution
void DynamicPlanner::setJointsLimits(const std::vector<std::string>& joints_names,
                                     const std::vector<double>& max_angles,
                                     const std::vector<double>& min_angles)
{
  // Check the consistency of the user inputs
  // If the assumption is false, the assert function halts
  // the program and produces a diagnostic message
  assert(joints_names.size() == max_angles.size());
  assert(joints_names.size() == min_angles.size());
  // I WOULD CHANGE the above statement with a runtime log and go on
  // with the rest of the function if is not verified

  // Limit Change
  const auto ja =
    robot_model_->getActiveJointModels(); // Get active joints from the model
  // Loop for each joint
  for (uint i = 0; i < ja.size(); i++)
  {
    const std::vector<std::string> robot_joints_names =
      ja[i]->getVariableNames(); // Get joints names
    for (uint j = 0; j < robot_joints_names.size();
         j++) // This should not be just one per joint -> Soft doubt NOMI NON IN ORDINE
              // CON I NUMERI DI GIUNTI
      for (uint k = 0; k < joints_names.size(); k++)
      {
        // If one of the joint names passed to the function is found in the robot model,
        // go on
        if (joints_names[k] != robot_joints_names[j])
          continue;
        moveit::core::VariableBounds limit = ja[i]->getVariableBounds(
          robot_joints_names[j]);            // Get the limits of the i-th active joint
        limit.max_position_ = max_angles[k]; // Max boundary set
        limit.min_position_ = min_angles[k]; // Min boundary set
        robot_model_
          ->getJointModel(robot_joints_names[j]) // Update the model with new bounds for
                                                 // the selected joint
          ->setVariableBounds(robot_joints_names[j], limit);
        break;
      }
  }
}

// Set the boolean value which activates the goal constraint on the wrist to the planner
void DynamicPlanner::setWristGoalConstraints(const bool value)
{
  wrist_goal_constraints_ = value;
  if (value)
    ROS_INFO("Adding wrist constraints to goal pose");
  else
    ROS_INFO("Removing wrist constraints from goal pose");
}

// Set path constraints as given by the input arg
/*
  Constraints : {name, JointConstraint[], PositionConstraint[], OrientationConstraint[],
  VisibilityConstraint[]} JointConstraint:       joint_name, [pos-tol_down, pos+tol_up],
  weight of importance (woi) PositionConstraint:    h, link_name, target_point_offset,
  constraint_region, woi OrientationConstraint: h, desired_orient, link_name, abs_x_tol,
  abs_y_tol, abs_z_tol, woi VisibilityConstraint:  never mind
*/
void DynamicPlanner::setPathConstraint(const moveit_msgs::Constraints constraints)
{
  params_.path_constraints = constraints;
  ROS_INFO("Enabled %s path constraints", constraints.name.c_str());
}

// Clear all path constraints
void DynamicPlanner::clearPathConstraint()
{
  params_.path_constraints.name.clear();
  params_.path_constraints.joint_constraints.clear();
  params_.path_constraints.position_constraints.clear();
  params_.path_constraints.orientation_constraints.clear();
  params_.path_constraints.visibility_constraints.clear();
  ROS_INFO("Disabled path constraints");
}

// Robot collision model padding
void DynamicPlanner::setPadding(const double link_padding)
{
  if (planning_scene_ == nullptr)
  {
    ROS_WARN("Cannot set link padding: planner is not initialized yet!");
    return;
  }
  auto coll =
    planning_scene_
      ->getCollisionRobotNonConst(); // Get the representation of the collision robot
  coll->setPadding(link_padding);    // Set a padding zone around the robot
  planning_scene_->propogateRobotPadding(); // Update the planning scene
  ROS_INFO("Setting link padding to %.3f [m]", link_padding);
}

// Robot collision model scaling
void DynamicPlanner::setScale(const double link_scale)
{
  if (planning_scene_ == nullptr)
  {
    ROS_WARN("Cannot set link scale: planner is not initialized yet!");
    return;
  }
  auto coll =
    planning_scene_
      ->getCollisionRobotNonConst(); // Get the representation of the collision robot
  coll->setScale(link_scale);        // Set a scale zone around the robot
  planning_scene_->propogateRobotPadding(); // Update the planning scene
  ROS_INFO("Setting link scale to %.3f", link_scale);
}

// Set sim mode: true for simulation, false for debug
void DynamicPlanner::setSimMode(const bool value) { sim_ = value; }

/*
Before seeing public planners, it's better to look at the composition of the
moveit_msgs/RobotState Message
  sensor_msgs/MultiDOFJointState multi_dof_joint_state  : not of our interest
  AttachedCollisionObject[] attached_collision_objects  : nothing new
  bool is_diff                                          : always false for us
  sensor_msgs/JointState joint_state
    std_msgs/Header header
    string[] name
    float64[] position
    float64[] velocity
    float64[] effort
*/

// PUBLIC PLANNERS: THEY DIFFER ON THE ARG GIVEN BY THE USER

// Planner V1 -> input: vector of joints final positions
void DynamicPlanner::plan(const std::vector<double>& final_position)
{
  // Planner V2 is called here
  plan(final_position, planning_group_name1_); // string of joint_model_group_name
}

// Planner V2 -> input: as V1 + joint planning group name
// WE SHOULD REPLACE THE FOLLOWING FUNCTION CODE WITH THESE TWO LINES !!!!!!!!!!  YES
// *robot_state_ = planning_scene_->getCurrentState();
// plan(final_position,joint_model_group_name,robot_state_)
void DynamicPlanner::plan(const std::vector<double>& final_position,
                          const std::string& joint_model_group_name)
{
  // Set global class variables as indicated by the function inputs
  final_position_ = final_position;
  planning_space_ = JOINTS_SPACE; // Planning in the joint 6D space
  planning_group_ = joint_model_group_name;

  // UPDATE INITIAL ROBOT STATE FOR PLANNING REQUEST
  *robot_state_ = planning_scene_->getCurrentState();
  // Set joint values for the joint model group 1
  robot_state_->setJointGroupPositions(
    joint_model_group1_, joints_values_group1_); // BEFORE PLANNING SET ROBOT STATE FROM
                                                 // JOITN STATE SUBCRIBER CALLBACK
  // If there's a planning group 2, overwrite joint values
  if (!planning_group_name2_.empty())
    robot_state_->setJointGroupPositions(joint_model_group2_, joints_values_group2_);
  // If there's a gripper, overwrite joints+gripper values
  if (!gripper_name_.empty())
    robot_state_->setJointGroupPositions(joint_model_gripper_, joints_values_gripper_);
  // Update planning request with planning settings and current robot state
  request_.group_name                      = joint_model_group_name;
  request_.planner_id                      = params_.name;
  request_.allowed_planning_time           = params_.planning_time;
  request_.num_planning_attempts           = params_.num_attempts;
  request_.max_velocity_scaling_factor     = params_.vel_factor;
  request_.max_acceleration_scaling_factor = params_.acc_factor;
  request_.path_constraints                = params_.path_constraints;
  robot_state::robotStateToRobotStateMsg(*robot_state_, request_.start_state);

  // BUILD GOAL FROM GOAL STATE
  // Goal state local variable created with the class constructor
  robot_state::RobotState goal_state(*robot_state_);
  // Specify the final position for a given joint model group to the goal state
  goal_state.setJointGroupPositions(joint_model_group_name, final_position_);
  // Create a constraint joints goal
  moveit_msgs::Constraints joints_goal = kinematic_constraints::constructGoalConstraints(
    goal_state, // robot goal state, already filled in
    robot_model_->getJointModelGroup(joint_model_group_name),
    0.0001,  // below radians tolerance per each joint
    0.0001); // above radians tolerance per each joint

  plan(joints_goal,
       true); // Calling dynamic planner private function for single joint goal
}

// Planner V3 -> input: as V2 + given robot state
// THE FOLLOWING FUNCTION STUCTURE IS THE SAME AS THE V2, ONLY ROBOT STATE CHANGES
void DynamicPlanner::plan(const std::vector<double>& final_position,
                          const std::string& joint_model_group_name,
                          const robot_state::RobotState& robot_state)
{
  // Set global class variables as indicated by the function inputs
  final_position_ = final_position;
  planning_space_ = JOINTS_SPACE;
  planning_group_ = joint_model_group_name;

  // UPDATE INITIAL ROBOT STATE FOR PLANNING REQUEST
  *robot_state_ = robot_state;
  // Set joint values for the joint model group 1
  if (planning_group_ != planning_group_name1_)
    robot_state_->setJointGroupPositions(joint_model_group1_, joints_values_group1_);
  // If there's a planning group 2, overwrite joint values
  if (planning_group_ != planning_group_name2_ && !planning_group_name2_.empty())
    robot_state_->setJointGroupPositions(joint_model_group2_, joints_values_group2_);
  // If there's a gripper, overwrite joints+gripper values
  if (!gripper_name_.empty())
    robot_state_->setJointGroupPositions(joint_model_gripper_, joints_values_gripper_);
  // Update planning request with planning settings and current robot state
  robot_state::robotStateToRobotStateMsg(*robot_state_, request_.start_state);
  request_.group_name                      = joint_model_group_name;
  request_.planner_id                      = params_.name;
  request_.allowed_planning_time           = params_.planning_time;
  request_.num_planning_attempts           = params_.num_attempts;
  request_.max_velocity_scaling_factor     = params_.vel_factor;
  request_.max_acceleration_scaling_factor = params_.acc_factor;
  request_.path_constraints                = params_.path_constraints;

  // BUILD GOAL FROM GOAL STATE
  // Goal state local variable created with the class constructor
  robot_state::RobotState goal_state(*robot_state_);
  // Specify the final position for a given joint model group to the goal state
  goal_state.setJointGroupPositions(joint_model_group_name, final_position_);
  // Create a constraint joints goal
  moveit_msgs::Constraints joints_goal = kinematic_constraints::constructGoalConstraints(
    goal_state, // robot goal state, already filled in
    robot_model_->getJointModelGroup(joint_model_group_name),
    0.001, // below radians tolerance per each joint
    0.01); // above radians tolerance per each joint -> THIS IS DIFFERENT, IS IT CORRECT
           // PROBABILMENTE EERRROE OK

  plan(joints_goal, false);
}

// Planner V4 -> input: multiple joints final positions
void DynamicPlanner::plan(const std::vector<std::vector<double>>& positions)
{
  // Planner V5 is called here
  plan(positions, planning_group_name1_);
}

// Planner V5 -> input: as V4 + joint planning group name
// THIS IS THE ONE FUNCTIONS THAT CALLS THE PLANNER plan(goal, trajectory)
void DynamicPlanner::plan(const std::vector<std::vector<double>>& positions,
                          const std::string& joint_model_group_name)
{
  // Get the last position as final joint goal
  final_position_ = positions.back();
  planning_space_ = JOINTS_SPACE;
  planning_group_ = joint_model_group_name;

  // Update initial robot state for planning request
  *robot_state_ = planning_scene_->getCurrentState();
  robot_state_->setJointGroupPositions(joint_model_group1_, joints_values_group1_);
  if (!planning_group_name2_.empty())
    robot_state_->setJointGroupPositions(joint_model_group2_, joints_values_group2_);
  // Update planning settings
  request_.group_name                      = joint_model_group_name;
  request_.planner_id                      = params_.name;
  request_.allowed_planning_time           = params_.planning_time;
  request_.num_planning_attempts           = params_.num_attempts;
  request_.max_velocity_scaling_factor     = params_.vel_factor;
  request_.max_acceleration_scaling_factor = params_.acc_factor;
  request_.path_constraints                = params_.path_constraints;
  robot_state::robotStateToRobotStateMsg(*robot_state_, request_.start_state);

  // Create a vector of multiple constraints as goals
  std::vector<moveit_msgs::Constraints> goals;
  // For each goal of the array 'positions' given by the user
  for (const auto& position : positions)
  {
    // Get robot the robot state associated to the position of the current iteration
    robot_state_->setJointGroupPositions(joint_model_group_name, position);
    // Add a kinematic constraint with HIGHER TOLERANCE than Planner V3
    goals.push_back(kinematic_constraints::constructGoalConstraints(
      *robot_state_, robot_model_->getJointModelGroup(joint_model_group_name), 0.1, 0.1));
  }

  plan(goals,
       trajectory_); // Calling dynamic planner private function for multiple joint goals

  // WHY THIS VARIABLE 'trajectory_' LOOKS ME EMPTY ?
}

// Planner V6 -> input: 3D carthesian final pose, link_name
void DynamicPlanner::plan(const geometry_msgs::PoseStamped& final_pose,
                          const std::string& link_name)
{
  // The planner V7 is called here
  plan(final_pose, link_name, planning_group_name1_);
}

// Planner V7 -> inputs: as V6 + joint planning group name
// THE FOLLOWING FUNCTION CODE SHOULD BE CHANGED WITH THESE TWO LINES YEASH OK
// *robot_state_ = planning_scene_->getCurrentState();
// plan(final_pose,link_name,joint_model_group_name,robot_state)
void DynamicPlanner::plan(const geometry_msgs::PoseStamped& final_pose,
                          const std::string& link_name,
                          const std::string& joint_model_group_name)
{
  // Update class global variables
  final_pose_         = final_pose;
  planning_space_     = CARTESIAN_SPACE;
  planning_link_name_ = link_name;
  planning_group_     = joint_model_group_name;

  // Fill the header frame with the default value "world"
  if (final_pose_.header.frame_id == "")
    final_pose_.header.frame_id = "world";

  // UPDATE INITIAL ROBOT STATE FOR PLANNING REQUEST -> THIS SECTION SHOULD BE A SEPARATE
  // FUNCTION
  *robot_state_ = planning_scene_->getCurrentState();
  // Set joint values for the joint model group 1
  robot_state_->setJointGroupPositions(joint_model_group1_, joints_values_group1_);
  // If there's a planning group 2, overwrite joint values
  if (!planning_group_name2_.empty())
    robot_state_->setJointGroupPositions(joint_model_group2_, joints_values_group2_);
  // If there's a gripper+joint planning group, overwrite joint values
  if (!gripper_name_.empty())
    robot_state_->setJointGroupPositions(joint_model_gripper_, joints_values_gripper_);
  // Update planning request with planning settings and current robot state
  request_.group_name                      = joint_model_group_name;
  request_.planner_id                      = params_.name;
  request_.allowed_planning_time           = params_.planning_time;
  request_.num_planning_attempts           = params_.num_attempts;
  request_.max_velocity_scaling_factor     = params_.vel_factor;
  request_.max_acceleration_scaling_factor = params_.acc_factor;
  request_.path_constraints                = params_.path_constraints;
  robot_state::robotStateToRobotStateMsg(*robot_state_, request_.start_state);

  // Create a 3D goal constraint
  goal_ = kinematic_constraints::constructGoalConstraints(
    link_name,   // The link name for both constraints
    final_pose_, // The pose stamped to be used for the target region
    0.001, // TOLERANCE POS: the dimension of the sphere associated with the target region
           // of the PositionConstraint
    0.001); // TOLERANCE ANGLE: the value to assign to the absolute tolerances of the
            // OrientationConstraint NOTE: THIS LAST VALUE CAN BE VERY STRICT !!!!!

  // Add a wrist goal constraint if specified by the user
  if (wrist_goal_constraints_)
  {
    // Set a new joint constraint
    moveit_msgs::JointConstraint wrist_constraint;
    wrist_constraint.joint_name      = "arm2_wrist_1_joint";
    wrist_constraint.weight          = 1;
    wrist_constraint.position        = -1.6;
    wrist_constraint.tolerance_above = 0.4;
    wrist_constraint.tolerance_below = wrist_constraint.tolerance_above;
    // Add the joint constraint to do goal constraints list
    goal_.joint_constraints.push_back(wrist_constraint);
  }

  plan(goal_, true); // Call the dynamic planner private function for a single goal
}

// Planner V8 -> inputs: as V7 + robot state
void DynamicPlanner::plan(const geometry_msgs::PoseStamped& final_pose,
                          const std::string& link_name,
                          const std::string& joint_model_group_name,
                          const robot_state::RobotState& robot_state)
{
  // Update class global variables
  final_pose_         = final_pose;
  planning_space_     = CARTESIAN_SPACE;
  planning_link_name_ = link_name;
  planning_group_     = joint_model_group_name;

  // Fill the header frame with the default value "world"
  if (final_pose_.header.frame_id == "")
    final_pose_.header.frame_id = "world";

  // UPDATE INITIAL ROBOT STATE FOR PLANNING REQUEST -> THIS SECTION SHOULD BE A SEPARATE
  // FUNCTION
  *robot_state_ = robot_state;
  // Set joint values for the joint model group 1
  if (planning_group_ != planning_group_name1_)
    robot_state_->setJointGroupPositions(joint_model_group1_, joints_values_group1_);
  // If there's a planning group 2, overwrite joint values
  if (planning_group_ != planning_group_name2_ && !planning_group_name2_.empty())
    robot_state_->setJointGroupPositions(joint_model_group2_, joints_values_group2_);
  // If there's a gripper+joint planning group, overwrite joint values
  if (!gripper_name_.empty())
    robot_state_->setJointGroupPositions(joint_model_gripper_, joints_values_gripper_);
  // Update planning request with planning settings and current robot state
  robot_state::robotStateToRobotStateMsg(*robot_state_, request_.start_state);
  request_.group_name                      = joint_model_group_name;
  request_.planner_id                      = params_.name;
  request_.allowed_planning_time           = params_.planning_time;
  request_.num_planning_attempts           = params_.num_attempts;
  request_.max_velocity_scaling_factor     = params_.vel_factor;
  request_.max_acceleration_scaling_factor = params_.acc_factor;
  request_.path_constraints                = params_.path_constraints;

  // Create a 3D goal constraint
  goal_ = kinematic_constraints::constructGoalConstraints(
    link_name,   // The link name for both constraints
    final_pose_, // The pose stamped to be used for the target region
    0.001, // TOLERANCE POS: the dimension of the sphere associated with the target region
           // of the PositionConstraint
    0.001); // TOLERANCE ANGLE: the value to assign to the absolute tolerances of the
            // OrientationConstraint NOTE: THIS LAST VALUE CAN BE VERY STRICT !!!!!

  // Add a wrist goal constraint if specified by the user
  if (wrist_goal_constraints_)
  {
    // Set a new joint constraint
    moveit_msgs::JointConstraint wrist_constraint;
    wrist_constraint.joint_name      = "arm2_wrist_1_joint";
    wrist_constraint.weight          = 1;
    wrist_constraint.position        = -1.6;
    wrist_constraint.tolerance_above = 0.4;
    wrist_constraint.tolerance_below = wrist_constraint.tolerance_above;
    // Add the joint constraint to do goal constraints list
    goal_.joint_constraints.push_back(wrist_constraint);
  }
  plan(goal_, false); // Call the dynamic planner private function for a single goal
}

// THE LAST VERSION SHOULD MISS: THE CORRESPONDING PLANNER IN THE 3D SPACE OF THE V5

// THIS FUNCTION IS NEVER CALLED BY ANOTHER FUNCTION IN THIS CLASS
// SHOULD IT BE CALLED TOGETHER WITH THE SPINNER
// OK IF YOU DON'T WANT THE DYNAMIC
// Recompute trajectory if an obstacle is found on the path
void DynamicPlanner::checkTrajectory()
{
  // If another functions has told that there is an obstruction along the path
  if (obstruction_)
  {
    // Try to complete the interrupted trajectory when the dynamic obstacle moves away
    switch (planning_space_)
    {
      case JOINTS_SPACE:
        // Try to get a new feasible joint trajectory (maybe we are lucky and the dynamic
        // obstacle has moved away from our trajectory)
        plan(final_position_, planning_group_);
        // Planner V2 -> uses current robot state as *robot_state_ =
        // planning_scene_->getCurrentState();
        break;
      case CARTESIAN_SPACE:
        // Try to get a new feasible   3D  trajectory (maybe we are lucky and the dynamic
        // obstacle has moved away from our trajectory)
        plan(final_pose_, planning_link_name_, planning_group_);
        // Planner V7 -> uses current robot state as *robot_state_ =
        // planning_scene_->getCurrentState();
        break;
    }

    // The above 'switch' is called once, so 'plan' is called once
    // The 'plan' function modifies the 'success', which is the negative boolean of
    // 'obstruction' If successfull, the 'plan' function fills the trajectory_ variable
    obstruction_ = !success_;
    return;
  }

  // If previous planning has not been successfull -> but it doensn't enter here in this
  // iteration ? Soft doubt
  if (!success_)
  {
    ROS_WARN_THROTTLE(3, "No feasible trajectory pending.");
    return;
  }

  // IF THERE IS NOT A KNOWN OBSTRUCTION OR A PREVIOUS OR A PREVIOUS PLANNING HAS NOT BEEN
  // SUCCESSFULL, ABOVE CODE IS NOT EXECUTED AND checkTrajectory() STARTS HERE YES OK

  // If current trajectory point is higher than the trajectory computed size, no more
  // trajectory computation is needed
  if (trajpoint_ >=
      (trajectory_.joint_trajectory.points.size() - 1)) // PUNTI DELLA TRAIETTORIA OKKKK
  {
    ROS_WARN_THROTTLE(3, "Trajectory complete or no trajectory pending.");
    return;
  }

  // IF WE ARRIVED HERE WITH THIS CODE, WE THINK THERE IS NO OBSTRUCTION AND WE TRY TO
  // FIND IF THERE IS A DYNAMIC OBSTACLE ALONG THE PATH

  // Create a new moveit object robot trajectory using the robot trajectory constructor
  moveit_msgs::RobotTrajectory robot_trajectory(trajectory_);
  ulong size  = robot_trajectory.joint_trajectory.points.size();
  ulong bound = std::min(size / 3, size - trajpoint_); // HEURISTICS OK

  // Get current robot state and copy it into the local variable robot_state (WITHOUT _
  // !!!)
  robot_state::RobotState robot_state = *robot_state_;

  // Iterate between current trajectory point and the given bound
  for (ulong i = trajpoint_; i < trajpoint_ + bound; ++i)
  {
    // Update Joint Positions as they were the current robot state for each iteration
    // so isStateColliding() function can be used to check a future collision
    robot_state.setJointGroupPositions(
      planning_group_, robot_trajectory.joint_trajectory.points[i].positions);

    // If future collisions are computed, enter the block, else the checkTrajectory()
    // functions ENDS
    if (planning_scene_->isStateColliding(robot_state, planning_group_, false))
    {
      ROS_WARN("Detected future collision with planning group %s in current trajectory: "
               "replanning..",
               planning_group_.c_str());
      // Store the state related to the collision incoming
      invalid_state_ = i;
      // HEURISTICS OK Take a trajectory point a little bit before the collision point
      ulong k = (invalid_state_ - trajpoint_) / 2;
      // ERASE the trajectory between point k (near before the collision) and the end
      robot_trajectory.joint_trajectory.points.erase(
        robot_trajectory.joint_trajectory.points.begin() +
          static_cast<long>(invalid_state_ - k),
        robot_trajectory.joint_trajectory.points.end());
      // Store the new trajectory (just cut) in the robot state
      moveit::core::jointTrajPointToRobotState(
        robot_trajectory.joint_trajectory,
        robot_trajectory.joint_trajectory.points.size() - 1, robot_state);
      // Rapidly try planning the first time (on-the-fly) -> CAN THIS SWITCH BE A NEW
      // FUNCTION YES OK
      switch (planning_space_)
      {
        case JOINTS_SPACE:
          plan(final_position_, planning_group_, robot_state);
          // Planner V3 -> uses a user defined robot state (new traj, old final goal)
          break;
        case CARTESIAN_SPACE:
          plan(final_pose_, planning_link_name_, planning_group_, robot_state);
          // Planner V8 -> uses a user defined robot state (new traj, old final goal)
          break;
      }

      // If above planning has been successfull
      if (success_)
        // Get new overall trajectory, since the planning was between point k and goal,
        // but we want from start to goal
        merge(robot_trajectory); // VELOCITA' E ACCELERAZIONE CONTINUEEE OKKK
      // Remember that robot_trajectory is a local variable within checkTrajectory()
      // function but within the above function 'merge', the new part of the trajectory
      // from k to goal is merged with the initial part of the trajectory and put into the
      // global class variable trajectory_
      else // If above planning has not been successfull
      {
        // The failed planning triggered a stop command to be published so we need to
        // recompute a new trajectory
        obstruction_ = true;
        ROS_WARN("Obstruction found");

        // Given that obstruction_ variable is again true, the first part of the code of
        // this function will be executed at the next call of checkTrajectory() The
        // 'switch' block of planning will take robot_state_ (current) as default for
        // planning In other words, if planning from k to goal didn't work, we'll try
        // replanning from start to goal a completely different trajectory
      }
      break;
    }
  }
}

// Move Robot function! A JointState msg is published on the MoveIt! fake controller
// JointState should contain a trajectory already verified as feasible
/*
  JointState:
      Header    header
      string[]  joints_names
      float64[] position
      float64[] velocity
      float64[] effort
*/
void DynamicPlanner::moveRobot(const sensor_msgs::JointState& joint_states)
{
  joints_pub_.publish(joint_states);
}

/*
  moveit_msgs/RobotTrajectory/JointTrajectory/JointTrajectoryPoint[].msg :
      float64[] positions
      float64[] velocities
      float64[] accelerations
      float64[] effort
      duration  time_from_start
*/

// Move Robot function given a trajectory to compute
void DynamicPlanner::moveRobot(const moveit_msgs::RobotTrajectory& robot_trajectory)
{
  // Create a JointState empty variable for the fake controller publisher
  sensor_msgs::JointState trajectory_pose;
  // Fill the name of the joints
  trajectory_pose.name = robot_trajectory.joint_trajectory.joint_names;

  // MoveRobot function is called per each following point of the whole trajectory
  // WHY TO NOT GIVE TO MOVEIT CLIENT THE ENTIRE TRAJECTORY the corresponding of python
  // 'execute'  TO VISUALIZE RVIZ ALL MID POINTS OK
  for (const auto& traj_pt : robot_trajectory.joint_trajectory.points)
  {
    trajectory_pose.position = traj_pt.positions;
    // trajectory_pose.velocity = traj_pt.velocities;
    moveRobot(trajectory_pose);
    ros::Duration(robot_trajectory.joint_trajectory.points[1].time_from_start)
      .sleep(); // IL SAMLLE DURATION OK
  }
}

// Spin ROS -> SHOULDN'T WE INSERT A LOOP RATE HERE TO CXALL THEN THE SPIN RATE LOOP OK
void DynamicPlanner::spinner() { ros::spinOnce(); }

// Check if group1, group2 and gripperJoints are received from the planner
// so the dynamic planner can start working
bool DynamicPlanner::isReady() const
{
  // The following booleans are true when the three subs have read something from active
  // pubs
  return joints_group1_received_ && joints_group2_received_ && joints_gripper_received_;
}

//---------- PRIVATE FUNCTIONS -----------------------------------------------//

// Planner initialization
void DynamicPlanner::initialize(const double v_factor, const double a_factor)
{
  // Setup publishers (for output trajectory) ...
  joints_pub_ = nh_.advertise<sensor_msgs::JointState>(
    "/move_group/fake_controller_joint_states", 1000);
  trajectory_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/trajectory", 1000);
  stop_pub_       = nh_.advertise<std_msgs::Bool>("/stop_trajectory", 1);
  // ...and subscribers (for robot status update)
  joints_sub_ = nh_.subscribe("/joint_states", 50, &DynamicPlanner::jointsCallback, this);
  trajpoint_sub_ =
    nh_.subscribe("/trajectory_counter", 2, &DynamicPlanner::trajPointCallback, this);
  // WHY QUEUES SIZE IS NOT 1 YEAH PUT AT 1 OK

  // Setup planning scene and robot model
  robot_model_loader_ = robot_model_loader::RobotModelLoader("/robot_description");
  robot_model_        = robot_model_loader_.getModel();
  robot_state_ = robot_state::RobotStatePtr(new robot_state::RobotState(robot_model_));
  planning_scene_ =
    planning_scene::PlanningScenePtr(new planning_scene::PlanningScene(robot_model_));
  planning_pipeline_ =
    planning_pipeline::PlanningPipelinePtr(new planning_pipeline::PlanningPipeline(
      robot_model_, nh_, "planning_plugin", "request_adapters"));
  planning_pipeline_->checkSolutionPaths(false);
  request_.goal_constraints.resize(1); // set the number of goal constraint as default 1

  // Update planner parameters; velocity and acceleration
  params_.vel_factor = v_factor;
  params_.acc_factor = a_factor;

  // Setup visual tools to display planned trajectory on RViz
  visual_tools_ =
    moveit_visual_tools::MoveItVisualToolsPtr(new moveit_visual_tools::MoveItVisualTools(
      robot_model_->getModelFrame(), rviz_visual_tools::RVIZ_MARKER_TOPIC, robot_model_));
  visual_tools_->deleteAllMarkers();  // Tell Rviz to clear all markers on a display
  visual_tools_->loadRemoteControl(); // Load MoveIt control
  visual_tools_->trigger();           // Trigger visual tools init
}

/*
moveit::core::VariableBounds Struct Reference
    double 	max_position_
    double 	min_position_
    double 	max_velocity_
    double 	min_velocity_
    double 	max_acceleration_
    double 	min_acceleration_
    bool 	  position_bounded_
    bool 	  velocity_bounded_
    bool 	  acceleration_bounded_
*/

// TODO: check why this doesn't work -> DOES IT STILL DOESN'T WORK IT MAY WORK BUT DYNAMIC
// ADAPATION OF JOINTS LIMIT TO PERFORM WELL OKKK Joints position limits adapter
void DynamicPlanner::adaptJointsLimits()
{
  // Clear previous mapping
  original_joints_limits_map_.clear();

  // Get the list of the current joint model group actived within the planner
  auto ja = joint_model_group1_->getActiveJointModels(); // vector of Joint Models

  // Iterate over the active joints
  for (uint i = 0; i < ja.size(); i++)
  {
    // Read joint names
    auto robot_joints_names = ja[i]->getVariableNames();
    // WHY A JOINT SHOULD HAVE MORE THAN 1 VARIABLE OK TRY A COUT TO SEE

    // Iterate over active joints limits
    for (uint j = 0; j < robot_joints_names.size(); j++)

      // Iterate over active joints start position
      for (uint k = 0; k < request_.start_state.joint_state.name.size(); k++)
      {
        // Check if names in the motion planning request correspond to active model loaded
        if (request_.start_state.joint_state.name[k] != robot_joints_names[j])
          continue;

        // Iterate over active joints costraints
        for (uint w = 0; w < request_.goal_constraints[0].joint_constraints.size(); w++)
        {
          // If the considered name is within the joint costraints
          if (request_.goal_constraints[0].joint_constraints[w].joint_name !=
              robot_joints_names[j])
            continue;

          // Get currently setup joint limits
          moveit::core::VariableBounds limit =
            ja[i]->getVariableBounds(robot_joints_names[j]);

          // Compute new desired limit positions among two possibilities:
          // 1: joint current position
          // 2: position defined within the goal constraint
          double max_position =
            std::max(request_.start_state.joint_state.position[k],
                     request_.goal_constraints[0].joint_constraints[w].position);
          double min_position =
            std::min(request_.start_state.joint_state.position[k],
                     request_.goal_constraints[0].joint_constraints[w].position);

          // If currently set positions are smaller/bigger than above computation, change
          // it
          if (limit.max_position_ < max_position)
            limit.max_position_ = max_position + 0.03;
          else if (limit.min_position_ > min_position)
            limit.min_position_ = min_position - 0.03;
          else
            break;

          // Store original joint limits
          original_joints_limits_map_[robot_joints_names[j]] =
            ja[i]->getVariableBounds(robot_joints_names[j]);

          // Temporarly update joint limits
          robot_model_->getJointModel(robot_joints_names[j])
            ->setVariableBounds(robot_joints_names[j], limit);

          // Log the results to the console
          ROS_WARN("Adapted joints limits to include start/stop positions:");
          ROS_WARN("- %s : [%.4f, %.4f] --> [%.4f, %.4f]", robot_joints_names[j].c_str(),
                   original_joints_limits_map_[robot_joints_names[j]].min_position_,
                   original_joints_limits_map_[robot_joints_names[j]].max_position_,
                   limit.min_position_, limit.max_position_);
          break;
        }
      }
  }
  /*
     if (original_joints_limits_map_.size() > 0)
     {
       // Re-build start from robot state with updated robot model (embedded in
       robot_state_) robot_state::robotStateToRobotStateMsg(*robot_state_,
       request_.start_state);
       // Re-build goal from goal state with updated robot model (embedded in
       robot_state_) robot_state::RobotState goal_state(*robot_state_); for (uint k = 0;
       k < request_.goal_constraints.size(); k++)
       {
         std::vector<double>
         goal_joints_pos(request_.goal_constraints[k].joint_constraints.size()); for
         (uint i = 0; i < goal_joints_pos.size(); ++i)
           goal_joints_pos[i] =
           request_.goal_constraints[k].joint_constraints[i].position;
         goal_state.setJointGroupPositions(joint_model_group_, goal_joints_pos);
         request_.goal_constraints[k] = kinematic_constraints::constructGoalConstraints(
           goal_state, joint_model_group_, 0.001, 0.01);
       }
       ROS_WARN("Re-built start and goal state");
     }
  */
}

// Contrary of the above function
void DynamicPlanner::restoreOriginalJointsLimits()
{
  // If the joints limits have been changed fro the user ...
  if (original_joints_limits_map_.size() > 0)
    ROS_WARN("Restoring original joints limits");
  // ... restore them
  for (const auto& el : original_joints_limits_map_)
    robot_model_->getJointModel(el.first)->setVariableBounds(el.first, el.second);
}

/*
moveit_msgs::RobotTrajectory  robot_trajectory
  trajectory_msgs/JointTrajectory joint_trajectory
    Header                  header
    string[]                joint_names
    JointTrajectoryPoint[]  points
      float64[] positions
      float64[] velocities
      float64[] accelerations
      float64[] effort
      duration  time_from_start
*/

// Merging robot trajectory
void DynamicPlanner::merge(moveit_msgs::RobotTrajectory& robot_trajectory)
{

  /*
    robot_trajectory (rt): input to the function, it's the trajectory we want to merge
    trajectory_ (t):       global class variable, it's the trajectory running now

    if size(rt) < size(t)
      t = [rt,t]
    else
      rt = [rt,t], t=rt da provare se c++ lo prende OK

    THE ACTUAL QUESTION IS ... WHY -> PUT ALWAYS THE NEW BEFORE  THE TRAJ YOU ALEADY HAVE
  */

  // If the trajectory input is SMALLER than the running one
  if (robot_trajectory.joint_trajectory.points.size() <
      trajectory_.joint_trajectory.points.size())

    // Iterate along all the points of the input trajectory
    for (uint k = 0; k < robot_trajectory.joint_trajectory.points.size(); ++k)
      // The new trajectory fills the global one from the end to its maximum possible
      trajectory_.joint_trajectory.points.insert(
        trajectory_.joint_trajectory.points.begin(), // index of the insert position
        robot_trajectory.joint_trajectory
          .points[robot_trajectory.joint_trajectory.points.size() -
                  (1 + k)]); // element to add

  // I would substitute the code of the above FOR loop with the following two lines
  // jt = robot_trajectory.joint_trajectory.points
  // t  = trajectory_.joint_trajectory.points
  // std::reverse(std::begin(jt),std::end(jt))
  // t = [jt,t]

  // If the trajectory input is BIGGER than the running one
  else
  {
    // Iterate along all the points of the global class trajectory
    for (const auto& traj_pt : trajectory_.joint_trajectory.points)
      robot_trajectory.joint_trajectory.points.push_back(traj_pt);
    trajectory_ = robot_trajectory;

    // I would substitute the above for loop with the following line
    // t = [rt,t]
  }

  trajectory_pub_.publish(trajectory_.joint_trajectory); // publish trajectory

  // Update trajectory visualization
  ROS_DEBUG("Publishing new trajectory of planning group %s on RViz topic %s..",
            planning_group_.c_str(), rviz_visual_tools::RVIZ_MARKER_TOPIC.c_str());
  visual_tools_->deleteAllMarkers();
  if (planning_group_ == planning_group_name1_)
    visual_tools_->publishTrajectoryLine(
      trajectory_,
      joint_model_group1_->getLinkModel(joint_model_group1_->getLinkModelNames().back()),
      joint_model_group1_);
  else
    visual_tools_->publishTrajectoryLine(
      trajectory_, joint_model_group2_->getLinkModel("arm2_wrist_3_link"),
      joint_model_group2_);
  visual_tools_->trigger();
}

// Check if the angle of the ee link fram exceeds the min_elevation_angle_ -> TO TAKE THE
// REEL FROM INSIDE POKKKK QUESTION: SHOULD WE CHANGE THE NAME OF THE "Eigen::Isometry3d"
// 'tf' so it's not misunderstood with the tf library OKKK
bool DynamicPlanner::checkElevationAngle(const robot_state::RobotStatePtr rs)
{
  // Get ee angle from the current robot state
  Eigen::Isometry3d tf = rs->getFrameTransform(ee_link_name_);

  // tf.matrix() extracts the 4x4 transformation matrix, so element (2,2)
  // is the rotation angle around y axis

  // Check execution: if out of the boundary, return false, else true
  if (tf.matrix()(2, 2) < std::sin(min_elevation_angle_))
  {
    ROS_DEBUG_STREAM("Elevation angle exceeds limits: "
                     << tf.matrix()(2, 2) << " < " << std::sin(min_elevation_angle_));
    return false;
  }

  return true;
}

// MY PERSONAL OPINION: robot_state_ SHOULD NOT BE USED IN THE FOLLOWING FUNCTION
// BUT IT SHOULD BE SUBSTITUTED BY A LOCAL VARIABLE OF THE SAME TYPE OR IT MAY
// HOLD THE VALUE OF THE LAST POSITION OF THE TRAJECTORY, however it should always
// contain the current value of robot state (containing current positions and velocities)

// Check if each point of a given trajectory passes checkElevationAngle()
bool DynamicPlanner::checkTrajectoryElevation(
  const moveit_msgs::RobotTrajectory& robot_trajectory)
{
  // Iterate over each point of the trajectory
  for (uint i = 0; i < robot_trajectory.joint_trajectory.points.size(); ++i)
  {
    // Set iterated point of the trajectory within the robot state
    robot_state_->setJointGroupPositions(
      planning_group_, robot_trajectory.joint_trajectory.points[i].positions);

    // Check if updated robot state passes the check
    if (!checkElevationAngle(robot_state_))
      return false;
  }
  return true;
}

void DynamicPlanner::plan(const moveit_msgs::Constraints& desired_goal, const bool send)
{
  // Set the goal to the MoveIt planning request
  request_.goal_constraints[0] = desired_goal;
  // Set current robot state in the planning scene
  planning_scene_->setCurrentState(*robot_state_);
  // Add collision and attached objects to the planning scene
  for (const auto& collision_object : collision_objects_)
    planning_scene_->processCollisionObjectMsg(collision_object);
  for (const auto& attached_object : attached_objects_)
    planning_scene_->processAttachedCollisionObjectMsg(attached_object);
  // adaptJointsLimits(); // make sure start and final poses are reachable

  // Reset
  success_     = false;
  obstruction_ = false;
  trajpoint_   = 0UL;

  // Until the maximum number of attempts is reached, try planning
  for (int num_tries = 0; num_tries < params_.num_attempts; ++num_tries)
  {
    planning_pipeline_->generatePlan(planning_scene_, request_, result_);

    // Check the result of the planning: if it has FAILED
    if (result_.error_code_.val != result_.error_code_.SUCCESS)
    {
      ROS_WARN("Could not compute plan successfully. Trying again (#%d attempt)..",
               num_tries + 1);
      continue;
    }
    // If planning was SUCCESSFULL
    else
    {
      // QUESTION: result_.trajectory_ goes into trajectory_ YES OKK
      result_.trajectory_->getRobotTrajectoryMsg(trajectory_);

      // Check if the user requests the elevation check AND if it is passed
      if (elevation_angle_check_ && !checkTrajectoryElevation(trajectory_))
      {
        ROS_WARN("Trajectory does not respect the orientation constraint. Trying again "
                 "(#%d attempt)..",
                 num_tries + 1);
        continue;
      }

      // If the code has passed above checks, planning can be considered successfull
      success_ = true;

      // Update trajectory visualization
      ROS_DEBUG("Publishing new trajectory of planning group %s on RViz topic %s..",
                planning_group_.c_str(), rviz_visual_tools::RVIZ_MARKER_TOPIC.c_str());
      visual_tools_->deleteAllMarkers();
      if (planning_group_ == planning_group_name1_)
        visual_tools_->publishTrajectoryLine(
          trajectory_,
          joint_model_group1_->getLinkModel(
            joint_model_group1_->getLinkModelNames().back()),
          joint_model_group1_);
      else
        visual_tools_->publishTrajectoryLine(
          trajectory_, joint_model_group2_->getLinkModel("arm2_wrist_3_link"),
          joint_model_group2_);
      visual_tools_->trigger();

      // If the user (input of the function) want to make the robot move (not just
      // display)
      if (send)
      {
        // For robot driver
        trajectory_pub_.publish(trajectory_.joint_trajectory);

        // For simulated robot
        if (sim_)
          moveRobot(trajectory_);
      }

      break;
    }
  }
  //  restoreOriginalJointsLimits();

  // If the planner has exceeded the number of attempts
  if (!success_)
  {
    ROS_ERROR("Too many attempts. I am not able to plan an admissable trajectory.");
    std_msgs::Bool stop;
    stop.data = true;
    stop_pub_.publish(stop);
  }
}

// Robot planner with goals and a given trajectory
void DynamicPlanner::plan(const std::vector<moveit_msgs::Constraints>& desired_goals,
                          moveit_msgs::RobotTrajectory& robot_trajectory)
{
  // Iterate over all the desired goals
  for (const auto& desired_goal : desired_goals)
  {
    // Plan to the currently considered desired goal
    plan(desired_goal, false);

    // If the planner has not been successfull, break the FOR loop
    if (!success_)
      break;

    // If the planner has been successful, go on merging trajectory
    merge(robot_trajectory);
    robot_trajectory                         = trajectory_;
    request_.planner_id                      = params_.name;
    request_.allowed_planning_time           = params_.planning_time;
    request_.num_planning_attempts           = params_.num_attempts;
    request_.max_velocity_scaling_factor     = params_.vel_factor;
    request_.max_acceleration_scaling_factor = params_.acc_factor;

    // The state msg to pass to the planner is taken from start (?) robot state // try to
    // comment OKKKK
    robot_state::robotStateToRobotStateMsg(*robot_state_, request_.start_state);
  }
}

//---------- ROS Callbacks ---------------------------------------------------//

// Subscriber function to '/trajectory_counter' topic
void DynamicPlanner::trajPointCallback(const std_msgs::Int32::ConstPtr& traj_point)
{
  // Returns the index of the current trajectory point data
  trajpoint_ = static_cast<ulong>(traj_point->data);
}

// Subscriber function to '/joint_states' topic
void DynamicPlanner::jointsCallback(const sensor_msgs::JointState::ConstPtr& joints_state)
{
  // Map to store couples joint name - joint values
  static std::unordered_map<std::string, double>::iterator it;
  uint counter_group1  = 0;
  uint counter_group2  = 0;
  uint counter_gripper = 0;
  // I DON'T UNDERSTAND WHY WE SHOULD USE THREE DIFFERENT GROUPS OF PLANNING OKKKK

  for (uint i = 0; i < joints_state->name.size(); i++)
  {
    // JOINT GRIPPER GROUP
    // Look for joints group names within joints current state
    it = joints_map_gripper_.find(joints_state->name[i]);
    // Exclude last link (gripper  from the search -> usero' ilmio gripper OKKKKKK
    if (it != joints_map_gripper_.end())
    {
      // At the second position of the iteration, insert current joint position
      it->second = joints_state->position[i];
      // Increment the number of joints receveid within the joints state subscriber
      counter_gripper++;
      // If we have reached the last joint of the group
      if (counter_gripper == joints_names_gripper_.size())
      {
        // Iterate over the joints
        for (uint k = 0; k < joints_names_gripper_.size(); k++)
          // Store the joints values from the joints map
          joints_values_gripper_[k] = joints_map_gripper_[joints_names_gripper_[k]];

        // Log gripper planning group
        ROS_INFO_ONCE("%s joints values received.", gripper_name_.c_str());

        // Confirm of msg reception
        joints_gripper_received_ = true;
      }
      continue;
    }

    // JOINT GROUP 1 -> same structure as above (only joints_map_group1_ and
    // planning_group_name1_ change). Can we put it into a new function ?
    it = joints_map_group1_.find(joints_state->name[i]);
    if (it != joints_map_group1_.end())
    {
      it->second = joints_state->position[i];
      counter_group1++;
      if (counter_group1 == joints_names_group1_.size())
      {
        for (uint k = 0; k < joints_names_group1_.size(); k++)
          joints_values_group1_[k] = joints_map_group1_[joints_names_group1_[k]];
        ROS_INFO_ONCE("%s joints values received.", planning_group_name1_.c_str());
        joints_group1_received_ = true;
      }
      continue;
    }

    // JOINT GROUP 2 -> same structure as above (only joints_map_group2_ and
    // planning_group_name2_ change). Can we put it into a new function ? We can delete
    // continue functions
    it = joints_map_group2_.find(joints_state->name[i]);
    if (it == joints_map_group2_.end())
      continue;
    it->second = joints_state->position[i];
    counter_group2++;
    if (counter_group2 < joints_names_group2_.size())
      continue;
    for (uint k = 0; k < joints_names_group2_.size(); k++)
      joints_values_group2_[k] = joints_map_group2_[joints_names_group2_[k]];
    ROS_INFO_ONCE("%s joints values received.", planning_group_name2_.c_str());
    joints_group2_received_ = true;
  }
}