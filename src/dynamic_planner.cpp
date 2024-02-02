/*
 * Software License Agreement (Apache Licence 2.0)
 *
 *  Copyright (c) [2024], [Andrea Pupa] [italo Almirante]
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   1. Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in
 *      the documentation and/or other materials provided with the
 *      distribution.
 *   3. The name of the author may not be used to endorse or promote
 *      products derived from this software without specific prior
 *      written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Author: [Andrea Pupa] [Italo Almirante]
 *  Created on: [2024-01-17]
 */

// CLASS SOURCE IMPLEMENTATION OF DYNAMIC PLANNER

// Import header file (and other useful libraries)
#include "dynamic_planner/dynamic_planner.h"

// ------------------ CLASS CONSTRUCTOR-------------------- //

// V1: 5 args, 8 private variables initializers
DynamicPlanner::DynamicPlanner(const std::string& manipulator_name,
                               const std::vector<std::string>& joints_name,
                               const double v_factor, const double a_factor,
                               const bool dynamic_behaviour)
  : nh_("~"), planning_group_name_(manipulator_name), joints_names_group_(joints_name),
    trajpoint_(0UL), success_(true), joints_group_received_(false),
    obstruction_(false), sim_(false), dynamic_behaviour_(dynamic_behaviour)
{
  // Load ROS pubs/subs, planning scene, robot model and state, visual tools and the planner
  initialize(v_factor, a_factor);

  // Initialize joints map for robot state update: per each joint name, set its value to 0
  for (const std::string& name : joints_names_group_)
    {joints_map_group_[name] = 0;}

  joints_values_group_.resize(joints_names_group_.size());  // Adjust joints values array size

  ros::Rate loop_rate(10);  // ROS loop rate definition, just to check readiness of ros pubs

  while (!isReady())
  {
    loop_rate.sleep();
    ros::spinOnce();
  }

  // Initialize robot state and its joints values
  joint_model_group_ = robot_state_->getJointModelGroup(planning_group_name_);
  robot_state_->setJointGroupPositions(joint_model_group_, joints_values_group_);

  // Move virtual robot to the initial position
  sensor_msgs::JointState initial_pose_msg;
  initial_pose_msg.name     = joints_names_group_;
  initial_pose_msg.position = joints_values_group_;
  moveRobot(initial_pose_msg);
}

//--------------------- PUBLIC FUNCTIONS -------------------------------------//

//--------------------- GETTER FUNCTIONS -------------------------------------//

std::vector<moveit_msgs::CollisionObject>& DynamicPlanner::getCollisionObjects()
{
  return collision_objects_;
}

std::vector<moveit_msgs::AttachedCollisionObject>& DynamicPlanner::getAttachedCollisionObjects()
{
  return attached_objects_;
}

moveit::planning_interface::PlanningSceneInterface& DynamicPlanner::getPlanningSceneInterface()
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

const moveit_msgs::RobotTrajectory DynamicPlanner::getTrajectory()
{
  return trajectory_;
}

const ulong DynamicPlanner::getTrajpoint()
{
  return trajpoint_;
}

const std::vector<moveit_msgs::Constraints> DynamicPlanner::getGoalsSeq()
{
  return goals_seq_;
}

const std::vector<double> DynamicPlanner::invKine(const geometry_msgs::PoseStamped& target_pose,
                                                  const std::string& link_name)
{
  // Create a copy of the kinematic state of robot model
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(robot_model_));
  std::vector<double> joint_values;

  // Create a copy of the current joint_model_group_
  const robot_state::JointModelGroup* joint_model_group = robot_model_->getJointModelGroup(planning_group_name_);

  // Perform inverse kinematics to find joint positions
  bool ik_success = kinematic_state->setFromIK(
    joint_model_group,    // group of joints to set
    target_pose.pose,     // the pose the last link in the chain needs to achieve
    0.1);                 // timeout,  default: 0.0 (no timeout)
  
  if (ik_success)
  {
    // Get joint values after successful IK
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);

    // Print joint values
    // ROS_INFO("Joint Values: ");
    // for (size_t i = 0; i < joint_values.size(); ++i)
    // {
    //   ROS_INFO("Joint %zu: %f", i, joint_values[i]);
    // }
  }
  return joint_values;
}

const geometry_msgs::PoseStamped DynamicPlanner::get_currentFKine()
{
  // Fill the JointState msg
  sensor_msgs::JointState current_joints_state;
  current_joints_state.header.seq = 1;
  current_joints_state.header.frame_id = "base_link";
  current_joints_state.header.stamp.sec = ros::Time::now().sec;
  current_joints_state.header.stamp.nsec = ros::Time::now().nsec;
  current_joints_state.name = joints_names_group_;
  // Update the msg with current joints values
  current_joints_state.position = {0.,0.,0.,0.,0.,0.};
  for (uint k = 0; k < joints_names_group_.size(); k++)
          {current_joints_state.position[k] = joints_values_group_[k];}
  // Compute FKINE
  return getFKine(current_joints_state);
}

// THE FOLLOWING FUNCTION DOESN'T WORK (error on: const Eigen::Affine3d& ... line)
const geometry_msgs::PoseStamped DynamicPlanner::getFKine(const sensor_msgs::JointState joint_state)
{
  // Create a copy of the current joint_model_group_
  const robot_model::JointModelGroup* joint_model_group = joint_model_group_;

  // Store joint values into a vector
  std::vector<double> joint_values = {0.,0.,0.,0.,0.,0.};
  for (unsigned int k = 0; k < 6; k++) {joint_values[k] = joint_state.position[k];}

  // Create a copy of the kinematic state of the required robot model
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(robot_model_));
  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);

  // Compute the forward kinematics -> THIS LINE LEADS TO AN ERROR
  const Eigen::Affine3d& end_effector_state = kinematic_state->getGlobalLinkTransform(ee_link_name_);
  
  Eigen::Vector3d translation_vector = end_effector_state.translation();
  Eigen::Vector3d rotation_angles = end_effector_state.rotation().eulerAngles(1, 2, 0); 

  // Print end-effector pose. Remember that this is in the model frame
  ROS_INFO_STREAM("Translation: \n" << translation_vector << "\n");
  ROS_INFO_STREAM("Rotation: \n"    << rotation_angles << "\n");

  // Create the quaternion
  tf2::Quaternion quaternion;
  quaternion.setRPY(rotation_angles[0], rotation_angles[1], rotation_angles[2]);

  // Fill the pose msg
  geometry_msgs::PoseStamped end_effector_pose;
  end_effector_pose.header.stamp.sec  = ros::Time::now().sec;
  end_effector_pose.header.stamp.nsec = ros::Time::now().nsec;
  end_effector_pose.header.frame_id   = "base_link";
  end_effector_pose.pose.position.x   = translation_vector[0];
  end_effector_pose.pose.position.y   = translation_vector[1];
  end_effector_pose.pose.position.z   = translation_vector[2];
  end_effector_pose.pose.orientation  = tf2::toMsg(quaternion);

  // return value
  return end_effector_pose;
}

const Eigen::MatrixXd DynamicPlanner::getJacobian()
{
    // Create a copy of the current joint_model_group_
  const robot_model::JointModelGroup* joint_model_group = joint_model_group_;

  // Create a copy of the kinematic state of the required robot model
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(robot_model_));

  // Get the Geometric Jacobian matrix of the manipulator
  Eigen::MatrixXd jacobian;
  Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
  kinematic_state->getJacobian(joint_model_group,
                               kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                               reference_point_position, jacobian);
  ROS_INFO_STREAM("Jacobian: \n" << jacobian << "\n");

  return jacobian;
}

//--------------------- SETTER FUNCTIONS -------------------------------------//

void DynamicPlanner::setParams(
                               const std::string& planner_id, const int attempts,
                               const double time, const double v_factor,
                               const double a_factor)
{
  params_ = DynamicPlannerParams(planner_id, attempts, time, v_factor, a_factor);
}

void DynamicPlanner::setParams(const DynamicPlannerParams& params)
{
  params_ = params;
}

// Set Joints Limits
void DynamicPlanner::setJointsLimits(
                                      const std::vector<std::string>& joints_names,
                                      const std::vector<double>&      max_angles,
                                      const std::vector<double>&      min_angles)
{
  // Check the consistency of the user inputs
  // If the assumption is false, the assert function halts 
  // the program and produces a diagnostic message
  assert(joints_names.size() == max_angles.size());
  assert(joints_names.size() == min_angles.size());

  // Limit Change
  const auto ja = robot_model_->getActiveJointModels(); // Get active joints from the model
  // Loop for each joint
  for (uint i = 0; i < ja.size(); i++)
  {
    // An iteration over robot model joints names and user given names is needed
    // since user (or robot) can give joints names in the wrong order
    const std::vector<std::string> robot_joints_names = ja[i]->getVariableNames();  // Get joints names from the model
    for (uint j = 0; j < robot_joints_names.size(); j++)                            // Get joints names from the arg
    {
      // Check all joints
      for (uint k = 0; k < joints_names.size(); k++)
      {
        // If one of the joint names passed to the function is found in the robot model, go on
        if (joints_names[k] == robot_joints_names[j])     
        {
          // Get the current limits of the i-th active joint
          moveit::core::VariableBounds limit = ja[i]->getVariableBounds(robot_joints_names[j]);  
          limit.max_position_ = max_angles[k];  // Max boundary set
          limit.min_position_ = min_angles[k];  // Min boundary set
          // Update the model with new bounds for the selected joint
          robot_model_->getJointModel(robot_joints_names[j])->setVariableBounds(robot_joints_names[j], limit);
          break;
        }
      }
    }
  }
}

/* Constraints msg type
  Constraints : {name, JointConstraint[], PositionConstraint[], OrientationConstraint[], VisibilityConstraint[]}
    JointConstraint:       joint_name, [pos-tol_down, pos+tol_up], weight of importance (woi)
    PositionConstraint:    h, link_name, target_point_offset, constraint_region, woi
    OrientationConstraint: h, desired_orient, link_name, abs_x_tol, abs_y_tol, abs_z_tol, woi
    VisibilityConstraint:  never mind
*/
// Set path constraints as given by the input arg
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
  auto coll = planning_scene_->getCollisionEnvNonConst();     // Get the representation of the collision robot
  coll->setPadding(link_padding);                             // Set a padding zone around the robot (in [m])
  planning_scene_->propogateRobotPadding();                   // Update the planning scene
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
  auto coll = planning_scene_->getCollisionEnvNonConst();     // Get the representation of the collision robot
  coll->setScale(link_scale);                                 // Set a scale zone around the robot (safe bounding box)
  planning_scene_->propogateRobotPadding();                   // Update the planning scene
  ROS_INFO("Setting link scale to %.3f", link_scale);
}

// Set sim mode: true for simulation, false for debug
void DynamicPlanner::setSimMode(const bool value) 
{
  sim_ = value;
}

/*
RobotState msg type
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

// --------------------- PUBLIC PLANNERS // -------------------------------------//

// Planner V1 -> input: vector of joints final positions
void DynamicPlanner::plan(const std::vector<double>& final_position)
{
  // Planner V2 is called here
  plan(final_position, planning_group_name_);  // string of joint_model_group_name
}

// Planner V2 -> input: as V1 + joint planning group name
void DynamicPlanner::plan(const std::vector<double>& final_position,
                          const std::string& joint_model_group_name)
{
  // Update initial robot stare for planning request
  *robot_state_ = planning_scene_->getCurrentState();

  // Call planner V3
  plan(final_position,joint_model_group_name,*robot_state_);
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

  // Set requested robot state
  *robot_state_ = robot_state;
  // Set joint values for the joint model group (current values are taken from the joint state subscriber)
  robot_state_->setJointGroupPositions(joint_model_group_, joints_values_group_);

  // Update planning request with desired planning settings and current robot state
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
  // Create a constrained joints goal 
  moveit_msgs::Constraints joints_goal = kinematic_constraints::constructGoalConstraints(
    goal_state, // robot goal state, already filled in 
    robot_model_->getJointModelGroup(joint_model_group_name),
    0.0001,     // below radians tolerance per each joint 
    0.0001);    // above radians tolerance per each joint 

  plan(joints_goal, true);  // Calling dynamic planner private function for single joint goal  

}

// Planner V4 -> input: multiple joints final positions
void DynamicPlanner::plan(const std::vector<std::vector<double>>& positions)
{
  // Planner V5 is called here
  plan(positions, planning_group_name_,false);    // TODO: change true if merging function works
}

// TODO: verify if it actually works
// Planner V5 -> input: as V4 + joint planning group name
//            -> output: a plan(goal, trajectory), it's the only one with this output
void DynamicPlanner::plan(const std::vector<std::vector<double>>& positions,
                          const std::string& joint_model_group_name,
                          bool online_replanning)
{
  // Get the last position as final joint goal
  final_position_ = positions.back();
  planning_space_ = JOINTS_SPACE;
  planning_group_ = joint_model_group_name;

  // Update initial robot state for planning request (both useful if the robot is standing or if it is moving)
  *robot_state_ = planning_scene_->getCurrentState();
  robot_state_->setJointGroupPositions(joint_model_group_, joints_values_group_);

  // Planning request update
  request_.group_name                      = joint_model_group_name;
  request_.planner_id                      = params_.name;
  request_.allowed_planning_time           = params_.planning_time;
  request_.num_planning_attempts           = params_.num_attempts;
  request_.max_velocity_scaling_factor     = params_.vel_factor;
  request_.max_acceleration_scaling_factor = params_.acc_factor;
  request_.path_constraints                = params_.path_constraints;

  // If new goals should not be added to previous ones, clear goals sequence and old trajectory
  if (!online_replanning) {goals_seq_.clear(); trajectory_.joint_trajectory.points.clear();}

  // For each goal of the array 'positions' given by the user
  for (const auto& position : positions)
  {
    // Get the robot state associated to the position of the current iteration
    robot_state::RobotStatePtr pos_robot_state;
    pos_robot_state->setJointGroupPositions(joint_model_group_name, position);

    // Add a kinematic constraint with HIGHER TOLERANCE than Planner V3
    goals_seq_.push_back(kinematic_constraints::constructGoalConstraints(
      *pos_robot_state,   // robot state associated to the currently iterated position
      robot_model_->getJointModelGroup(joint_model_group_name), 
      0.01,               // below radians absolute tolerance
      0.01));             // upper radians absolute tolerance
  }

  // Calling dynamic planner private function for multiple joint goals
  plan(goals_seq_, trajectory_);

}

// Planner V6 -> input: 3D carthesian final pose, link_name
void DynamicPlanner::plan(const geometry_msgs::PoseStamped& final_pose,
                          const std::string& link_name)
{
  // The planner V7 is called here
  plan(final_pose, link_name, planning_group_name_);
}

// Planner V7 -> inputs: as V6 + joint planning group name
void DynamicPlanner::plan(const geometry_msgs::PoseStamped& final_pose,
                          const std::string& link_name,
                          const std::string& joint_model_group_name)
{
  // Update current robot state for planning
  *robot_state_ = planning_scene_->getCurrentState();
  
  // Call planner V8
  plan(final_pose,link_name,joint_model_group_name,*robot_state_);

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

  // Get robot state as requested by the user
  *robot_state_ = robot_state;
  // Set joint values for the joint model group
  robot_state_->setJointGroupPositions(joint_model_group_, joints_values_group_);
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
            link_name,    // The link name for both constraints 
            final_pose_,  // The pose stamped to be used for the target region
            0.001,        // TOLERANCE POS: the dimension of the sphere associated with the target region of the PositionConstraint
            0.01);        // TOLERANCE ANGLE: the value to assign to the absolute tolerances of the OrientationConstraint!

  // Call the dynamic planner private function for a single goal
  plan(goal_, true);

}

// Planner V9 -> inputs: vectors of 3D carthesian poses + link_name (usually the end effector
//            -> output: planner V5
void DynamicPlanner::plan(const std::vector<geometry_msgs::PoseStamped>& target_poses,
                          const std::string& link_name)
{
  // Vector of joints positions
  std::vector<std::vector<double>> joint_positions;

  // Iterate over each target pose
  for (const auto& target_pose : target_poses)
  {
    // Add the invertred joint position
    joint_positions.push_back(invKine(target_pose,ee_link_name_));  // TODO: check if the solution of InvKine doesn't hide multiple solutions
  }

  // Pass inverted positions to the V5 planner
  plan(joint_positions, planning_group_name_, false); // TODO: replace with true when we are sure of the replanning mode
}

// TODO: change comptletely this function when mover decoupling will be done
// Check trajectory function: individuate an invalide state and correct trajectory online
void DynamicPlanner::checkTrajectory()
{

  // Check if the user has set the dynamic behaviour within the constructor: if not so, block the function
  if (!dynamic_behaviour_)
  {
    return;
  }

  // If another functions has told that there is an obstruction along the path, try to compute a new feasible trajectory
  if (obstruction_)
  {
    // Try to complete the interrupted trajectory when the dynamic obstacle moves away
    switch (planning_space_)
    {
      case JOINTS_SPACE:
        // Try to get a new feasible joint trajectory (maybe we are lucky and the dynamic obstacle has moved away from our trajectory)
        plan(final_position_, planning_group_);                   
        // Planner V2 -> uses current robot state as *robot_state_ = planning_scene_->getCurrentState();
        break;
      case CARTESIAN_SPACE:
        // Try to get a new feasible   3D  trajectory (maybe we are lucky and the dynamic obstacle has moved away from our trajectory)
        plan(final_pose_, planning_link_name_, planning_group_);  
        // Planner V7 -> uses current robot state as *robot_state_ = planning_scene_->getCurrentState();
        break;
    }

    // The above 'switch' is called once, so 'plan' is called once
    // The 'plan' function modifies the 'success', so it becoms the negative boolean of 'obstruction'
    // If successfull, the 'plan' function fills the trajectory_ global class variable
    obstruction_ = !success_;
    return;
  }

  // If previous planning has not been successfull, stop planning and trajectory execution
  if (!success_)
  {
    ROS_WARN_THROTTLE(3, "No feasible trajectory pending.");
    return;
  }

  // If there's not a known obstruction and previous planning has been successfull, just check if something has changed
  // If between previous and current ROS spin, an obstacle is put along currently executing path, try to compute a new trajectory 

  // If current trajectory point is higher than the trajectory computed size, no more trajectory computation is needed
  if (trajpoint_ >= (trajectory_.joint_trajectory.points.size() - 1))
  {
    ROS_WARN_THROTTLE(3, "Trajectory complete or no trajectory pending.");
    return;
  }

  // If we arrived here with this code, we think there is no obstruction and we try
  // to find if there is a dynamic obstacle along the path

  // Create a new moveit object robot trajectory using the robot trajectory constructor
  moveit_msgs::RobotTrajectory robot_trajectory(trajectory_);   // trajectory_ value GOES INTO robot_trajectory
  ulong size  = robot_trajectory.joint_trajectory.points.size();// trajectory size
  ulong bound = std::min(size / 3, size - trajpoint_);          // heuristics: look a bit AHEAD of current trajectory point executing
  // Look for obstacles ahead of 1/3 of the trajectory length, or, if less the 1/3 of the trajectory is remained,
  // check all the remaining trajectory

  // Get current robot state and copy it into the local variable robot_state
  robot_state::RobotState robot_state = *robot_state_;

  // COULD ME MORE EFFICIENT TO UPDATE ITERATOR OF A CERTAIN STEP NOT UNIT ?
  // WHAT IS TRAJECTORY RESOLUTION? WHAT IS OBSTACLE RESOLUTION?
  // CAN WE CHECK TRAJECTORY ONLY IF WE GET THAT PLANNING SCENE IS ACTUALLY CHANGED?

  // Iterate between current trajectory point and the given bound 
  for (ulong i = trajpoint_; i < trajpoint_ + bound; ++i)
  {
    // Update Joint Positions into robot state for each iteration
    // so isStateColliding() function can be used to check a future collision
    robot_state.setJointGroupPositions(
      planning_group_, robot_trajectory.joint_trajectory.points[i].positions);

    // If future collisions are computed, enter the block, else the checkTrajectory() functions ENDS 
    if (planning_scene_->isStateColliding(robot_state, planning_group_, false))
    {
      ROS_WARN("Detected future collision with planning group %s in current trajectory: "
               "replanning..", planning_group_.c_str());

      // Store the state related to the collision incoming
      invalid_state_ = i;

      // Heuristics: take a trajectory point a little bit before the collision point
      ulong k = (invalid_state_ - trajpoint_) / 2;

      // ERASE the trajectory between point k (near before the collision) and the end
      robot_trajectory.joint_trajectory.points.erase(
        robot_trajectory.joint_trajectory.points.begin() + static_cast<long>(invalid_state_ - k),
        // above raw could be replaced by robot_trajectory.joint_trajectory.points[static_cast<long>((inv_state+trajpoint_)/2)]
        robot_trajectory.joint_trajectory.points.end());

      // Within robot_trajectory, the path remained goes from the start to THE MIDDLE POINT BETWEEN the invalid state and current trajpoint

      // Store the new trajectory (just cut) in the robot state
      moveit::core::jointTrajPointToRobotState( robot_trajectory.joint_trajectory,
                                                robot_trajectory.joint_trajectory.points.size() - 1, 
                                                robot_state);

      // Try to plan from the set robot state to the goal to find a feasible path that avoids the obstacle(s)
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

      // If above planning has been successfull, success_ became true and trajectory_ global class variable
      // has been filled with the path BETWEEN K AND THE END, while in robot_trajectory local variable there is
      // the path BETWEEN BEGIN AND K (obtained by previously cut entire old trajectory_)
      if (success_)
      {
        merge(robot_trajectory); 
      }        
      else  // If above planning has not been successfull
      {
        // We'll try replanning from start to goal a completely different trajectory 
        obstruction_ = true;
        ROS_WARN("Obstruction found");
      }
      break;
    }
  }
}

// JointState should contain a trajectory already verified as feasible
/*
  JointState:
      Header    header
      string[]  joints_names
      float64[] position
      float64[] velocity
      float64[] effort
*/

// Move Robot function! A JointState msg is published on the MoveIt! fake controller
void DynamicPlanner::moveRobot(const sensor_msgs::JointState& joint_states)
{
  joints_pub_.publish(joint_states);
}

/* moveit_msgs/RobotTrajectory/JointTrajectory/JointTrajectoryPoint[].msg :
      float64[] positions
      float64[] velocities
      float64[] accelerations
      float64[] effort
      duration  time_from_start
*/

// Move Robot function given a trajectory to compute -> THIS FUNCTION BLOCKS THE CODE RUNNING
// THIS SHOULD BE PUT IN A DIFFERENT NODE:: TODO
void DynamicPlanner::moveRobot(const moveit_msgs::RobotTrajectory& robot_trajectory)
{
  // Create a JointState empty variable for the fake controller publisher
  sensor_msgs::JointState trajectory_pose;
  // Fill the name of the joints
  trajectory_pose.name = robot_trajectory.joint_trajectory.joint_names;

  // MoveRobot function is called per each following point of the whole trajectory, to visualize each point on RViz
  for (const auto& traj_pt : robot_trajectory.joint_trajectory.points)
  {
    trajectory_pose.position = traj_pt.positions;
    // trajectory_pose.velocity = traj_pt.velocities;
    moveRobot(trajectory_pose);
    ros::Duration(robot_trajectory.joint_trajectory.points[1].time_from_start).sleep(); 
    // The above ros duration commands says that the sampling time a new trajpoint is given to the publisher 
    // It depends on the uniform sample duration made by the plugin
  }
}

// Spin ROS (the loop rate is set in the proper node)
void DynamicPlanner::spinner() 
{ 
  ros::spinOnce(); 
  checkTrajectory();
}

// Check if the planner has received group definition, so the dynamic planner can start working
bool DynamicPlanner::isReady() const
{
  // The following booleans are true when the three subs have read something from active pubs 
  return joints_group_received_;
}

//---------- PRIVATE FUNCTIONS -----------------------------------------------//

// Planner initialization
void DynamicPlanner::initialize(const double v_factor, const double a_factor)
{
  // Setup publishers (for output trajectory) ...
  joints_pub_     = nh_.advertise<sensor_msgs::JointState>(
                    "/move_group/fake_controller_joint_states", 1);  
  trajectory_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/trajectory", 1);
  stop_pub_       = nh_.advertise<std_msgs::Bool>("/stop_trajectory", 1);
  // ...and subscribers (for robot status update)
  joints_sub_     = nh_.subscribe("/joint_states", 1, &DynamicPlanner::jointsCallback, this);
  trajpoint_sub_ =  nh_.subscribe("/trajectory_counter", 1, &DynamicPlanner::trajPointCallback, this);

  // Setup planning scene and robot model
  robot_model_loader_ = robot_model_loader::RobotModelLoader("/robot_description");
  robot_model_        = robot_model_loader_.getModel();
  robot_state_        = robot_state::RobotStatePtr(new robot_state::RobotState(robot_model_));
  planning_scene_     = planning_scene::PlanningScenePtr(new planning_scene::PlanningScene(robot_model_));
  planning_pipeline_  = planning_pipeline::PlanningPipelinePtr(new planning_pipeline::PlanningPipeline(
                        robot_model_, nh_, "planning_plugin", "request_adapters"));

  planning_pipeline_->checkSolutionPaths(false);
  request_.goal_constraints.resize(1);

  // Update planner parameters; velocity and acceleration
  params_.vel_factor = v_factor;
  params_.acc_factor = a_factor;

  // Setup visual tools to display planned trajectory on RViz
  visual_tools_ = moveit_visual_tools::MoveItVisualToolsPtr(new moveit_visual_tools::MoveItVisualTools(
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

// Joints position limits adapter -> TODO: check if it works with a cout
void DynamicPlanner::adaptJointsLimits() 
{
  // Clear previous mapping
  original_joints_limits_map_.clear();

  // Get the list of the current joint model group actived within the planner
  auto ja = joint_model_group_->getActiveJointModels();

  // Iterate over the active joints
  for (uint i = 0; i < ja.size(); i++)
  {
    // Read joint names
    auto robot_joints_names = ja[i]->getVariableNames();    
    
    // Iterate over active joints names variabls
    for (uint j = 0; j < robot_joints_names.size(); j++)
      
      // Iterate over active joints start position
      for (uint k = 0; k < request_.start_state.joint_state.name.size(); k++)
      {
        // Check if names in the motion planning request correspond to active model loaded
        if (request_.start_state.joint_state.name[k] == robot_joints_names[j])
        {
          // Iterate over active joints costraints
          for (uint w = 0; w < request_.goal_constraints[0].joint_constraints.size(); w++)
          {
            // If the considered name is within the joint costraints (sequence of names can be different)
            if (request_.goal_constraints[0].joint_constraints[w].joint_name == robot_joints_names[j])
            {
              // Get currently setup joint limits
              moveit::core::VariableBounds limit = ja[i]->getVariableBounds(robot_joints_names[j]);

              // Compute new desired limit positions among two possibilities:
              // 1: joint current position
              // 2: position defined within the goal constraint
              double max_position = std::max(request_.start_state.joint_state.position[k],
                                    request_.goal_constraints[0].joint_constraints[w].position);
              double min_position = std::min(request_.start_state.joint_state.position[k],
                                    request_.goal_constraints[0].joint_constraints[w].position);

              // If currently set positions are smaller/bigger than above computation, change it
              if (limit.max_position_ < max_position)
                limit.max_position_ = max_position + 0.03;
              else if (limit.min_position_ > min_position)
                limit.min_position_ = min_position - 0.03;
              else
                break;

              // Store original joint limits
              original_joints_limits_map_[robot_joints_names[j]] = ja[i]->getVariableBounds(robot_joints_names[j]);
              
              // Temporarly update joint limits
              robot_model_->getJointModel(robot_joints_names[j])->setVariableBounds(robot_joints_names[j], limit);
              
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

// Merging a given robot trajectory with the preempted trajectory_
void DynamicPlanner::merge(moveit_msgs::RobotTrajectory& robot_trajectory)
{
  /* 
    robot_trajectory (rt): input to the function, it's the trajectory we want to merge
    trajectory_ (t):       global class variable, it's the trajectory running now

    if size(rt) < size(t)       # PUSH_FRONT
      t = [rt,t]
    else                        # PUSH_BACK
      rt = [rt,t], t=rt 
    # TODO: to test if c++ coding accepts this implementation as buffer

    The switching in size depends on the length to help fasten the code
  */

  // If the trajectory input is SMALLER than the running one
  if (robot_trajectory.joint_trajectory.points.size() < trajectory_.joint_trajectory.points.size())
  {
    // Iterate along all the points of the input trajectory
    for (uint k = 0; k < robot_trajectory.joint_trajectory.points.size(); ++k)
    {
      // The new trajectory fills the global one by putting the last element into the first index ITERATIVELY (pushes the former ones back)
      trajectory_.joint_trajectory.points.insert(
        trajectory_.joint_trajectory.points.begin(),                    // index of the insert position
        robot_trajectory.joint_trajectory.points[
          robot_trajectory.joint_trajectory.points.size() - (1 + k)]);  // element to add

      // I would substitute the code of the above FOR loop with the following two lines
      // Rt = robot_trajectory.joint_trajectory.points
      // t  = trajectory_.joint_trajectory.points
      // t = [Rt,t]

      // trajectory_.joint_trajectory.points = robot_trajectory.joint_trajectory.points.push_back(trajectory_.joint_trajectory.points)
      // or
      // trajectory_.joint_trajectory.points = [robot_trajectory.joint_trajectory.points,trajectory_.joint_trajectory.points] -> I BET ON THIS
    }
  }
  
  // If the trajectory input is BIGGER (or equal) than the running one (it never happens in this library)
  else
  {
    // Iterate along all the points of the global class trajectory
    for (const auto& traj_pt : trajectory_.joint_trajectory.points)
    {
      robot_trajectory.joint_trajectory.points.push_back(traj_pt);
    }
    trajectory_ = robot_trajectory;

    // TODO: I would substitute the above for loop with the following line
    // t = [rt,t]
  }  

  // TODO: try this one an comment all above code
  // trajectory_.joint_trajectory.points = [robot_trajectory.joint_trajectory.points,trajectory_.joint_trajectory.points]

  // Publish trajectory
  trajectory_pub_.publish(trajectory_.joint_trajectory);

  // Update trajectory visualization
  trajectoryVisualizer(trajectory_);
}

// Basic private planning function
void DynamicPlanner::plan(const moveit_msgs::Constraints& desired_goal, const bool send)
{
  // TODO: the following goal constraints request should be a push back 
  // (this vector should be cleared at the calling of the planning request)

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

  // Reset previous planning results
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
      success_ = true;

      // result_.trajectory_ goes into trajectory_ global class variable
      result_.trajectory_->getRobotTrajectoryMsg(trajectory_);      

      // TODO: the following lines (until before the break at line 1019) need to be deleted
      // Update trajectory visualization
      trajectoryVisualizer(trajectory_);

      // If the user (input of the function) want to make the robot move (not just display)
      if (send)
      {
        // For robot driver
        trajectory_pub_.publish(trajectory_.joint_trajectory);

        // For simulated robot
        if (sim_) {moveRobot(trajectory_);}
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

// Robot planner with vectors of goals and a given trajectory goal (made of a sequence of desired goals)
void DynamicPlanner::plan(const std::vector<moveit_msgs::Constraints>& desired_goals,
                          moveit_msgs::RobotTrajectory& robot_trajectory)
{
  // Iterate over all the desired goals
  for (const auto& desired_goal : desired_goals)
  {
    // Plan to the currently considered desired goal
    plan(desired_goal, false);
    // TODO: add another input to create a robot state which corresponds to the previous goal

    // If the planner has not been successfull, break the FOR loop
    if (!success_)
      break;

    // If the planner has been successful, trajectory is merged
    merge(robot_trajectory);
    robot_trajectory = trajectory_;
  }
}

// Visualize on RViz a given trajectory
void DynamicPlanner::trajectoryVisualizer(moveit_msgs::RobotTrajectory& robot_trajectory)
{
    ROS_DEBUG("Publishing new trajectory of planning group %s on RViz topic %s..",
                planning_group_.c_str(), rviz_visual_tools::RVIZ_MARKER_TOPIC.c_str());
    visual_tools_->deleteAllMarkers();
    if (planning_group_ == planning_group_name_)
      visual_tools_->publishTrajectoryLine(
        trajectory_,
        joint_model_group_->getLinkModel(joint_model_group_->getLinkModelNames().back()),
        joint_model_group_);
    visual_tools_->trigger();
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
  uint counter_group  = 0;

  for (uint i = 0; i < joints_state->name.size(); i++)
  {
    // Look for joints group names within joints current state
    it = joints_map_group_.find(joints_state->name[i]);
    // Exclude last link (gripper) from the search
    if (it != joints_map_group_.end())
    {
      // At the second position of the iteration, insert current joint position
      it->second = joints_state->position[i];
      // Increment the number of joints recevied from the joints state subscriber
      counter_group++;
      // If we have reached the last joint of the group
      if (counter_group == joints_names_group_.size())
      {
        // Iterate over the joints
        for (uint k = 0; k < joints_names_group_.size(); k++)
          // Store the joints values from the joints map
          {joints_values_group_[k] = joints_map_group_[joints_names_group_[k]];}

        // Log gripper planning group
        ROS_INFO_ONCE("%s joints values received.", planning_group_name_.c_str());
        joints_group_received_ = true;
      }
    }
  }
}