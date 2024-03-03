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

// CLASS HEADER IMLEMENTATION OF DYNAMIC PLANNER

#ifndef DYNAMIC_PLANNER_H
#define DYNAMIC_PLANNER_H

// IMPORT LIBRARIES

//C++ Libraries
#include <cmath>
#include <unordered_map> // https://en.cppreference.com/w/cpp/container/unordered_map

// ROS Libaries
#include <control_msgs/JointTrajectoryControllerState.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

// MoveIt! Libaries
#include <moveit/collision_detection/collision_tools.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/trajectory_processing/iterative_spline_parameterization.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

// Global constant definition of the number of joints 
#define JOINTS_NUM 6

// STRUCT definition of the parameters of the Dynamic Planner
  struct DynamicPlannerParams
  {
    std::string name     = "RRTConnect";        // name of the planner method (look up from list in ompl_planning.yaml)
    int num_attempts     = 5;                   // max number of attempts to find a trajectory
    double planning_time = 5;                   // maximum planning time in seconds (default 0: no limit)
    double vel_factor    = 99.;                 // velocity factor
    double acc_factor    = 99.;                 // acceleration factor
    moveit_msgs::Constraints path_constraints;  // moveit vector of path constraints

    // Struct constructors declaration for the dynamic planner params setup
    // V1: empty struct
    DynamicPlannerParams() {}
    // V2: passing args as initializers
    DynamicPlannerParams(const std::string& planner_id, const int attempts,
                        const double time, const double v_factor, const double a_factor)
      : name(planner_id), 
        num_attempts(attempts), 
        planning_time(time), 
        vel_factor(v_factor),
        acc_factor(a_factor)
    {}
  };

// CLASS DECLARATION FOR DYNAMIC PLANNER
class DynamicPlanner
{
public:

  // --------------------- PUBLIC CONSTRUCTOR ---------------------

    DynamicPlanner(const std::string& manipulator_name,
                  const std::vector<std::string>& joints_name, 
                  const double v_factor = 0.2,
                  const double a_factor = 0.2,
                  const bool dynamic_behaviour = false);

  // --------------------- PUBLIC FUNCTIONS ---------------------

    // MoveIt! gets planning scene and collision objects
      std::vector<moveit_msgs::CollisionObject>& getCollisionObjects();
      std::vector<moveit_msgs::AttachedCollisionObject>& getAttachedCollisionObjects();
      moveit::planning_interface::PlanningSceneInterface& getPlanningSceneInterface();
      const planning_scene::PlanningScenePtr getPlanningScenePtr();
      const moveit_visual_tools::MoveItVisualToolsPtr getVisualToolsPtr();

    // Get current trajectory state
      const moveit_msgs::RobotTrajectory getTrajectory();
      const ulong getTrajpoint();
      const std::vector<moveit_msgs::Constraints> getGoalsSeq();

    // Perform inverse or forward kinematics
      const std::vector<double> invKine(const geometry_msgs::PoseStamped& target_pose,
                                                       const std::string& link_name);
      const geometry_msgs::PoseStamped get_currentFKine();
      const geometry_msgs::PoseStamped getFKine(const sensor_msgs::JointState joint_states);
      const Eigen::MatrixXd getJacobian();

    // Dynamic planner parameters getter and setter
      DynamicPlannerParams getParams() const { return params_; }  // Getter is already implemented here!!!
      // The 'const' indicates that this member function does not modify the state of the object it is called on.

      void setParams(const std::string& planner_id, const int attempts, const double time,
                     const double v_factor, const double a_factor);
      void setParams(const DynamicPlannerParams& params);

      // Set joints limits as vectors of min-max angles
      void setJointsLimits(const std::vector<std::string>& joints_names,
                          const std::vector<double>&       max_angles,
                          const std::vector<double>&       min_angles);

      // Path and wrist constraints setup 
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
                const std::string&         joint_model_group_name);
      void plan(const std::vector<double>& final_position,
                const std::string&             joint_model_group_name,
                const robot_state::RobotState& robot_state);
      // Multiple JOINTS goals
      void plan(const std::vector<std::vector<double>>& positions,
                bool                                    online_replanning);
      void plan(const std::vector<std::vector<double>>& positions,
                const std::string&                      joint_model_group_name,
                bool                                    online_replanning);
      // Single POSITION goal (within 3D carthesian space, operative space)
      void plan(const geometry_msgs::PoseStamped& final_pose,
                const std::string&                link_name);
      void plan(const geometry_msgs::PoseStamped& final_pose,
                const std::string&                link_name,
                const std::string&                joint_model_group_name);
      void plan(const geometry_msgs::PoseStamped& final_pose,
                const std::string&                link_name,
                const std::string&                joint_model_group_name,
                const robot_state::RobotState&    robot_state);
      // Multiple POSITIONS goals (within 3D carthesian space, operative space)
      void plan(const std::vector<geometry_msgs::PoseStamped>& target_poses,
                const std::string&                             link_name);                
      // Cartesian planner
      double cartesianPlan(const std::vector<geometry_msgs::Pose>& waypoints,
                           const double                            eef_step);
      // Check trajectory feasibility as long as the dynamic object moves or enters/exits from the scene
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
  
private:

  // ---------------------  PRIVATE ENUMS -------------------
    // Planning space enum definition -> standard joints and cartesian spaces
    enum PlanningSpace : bool
    {
      JOINTS_SPACE,     // 0 if we didn't want to use the enum
      CARTESIAN_SPACE   // 1 if we didn't want to use the enum
    };

  // ---------------------  PRIVATE VARIABLES ------------------
    // ROS node, publishers and subscribers definition
      ros::NodeHandle nh_;            // node to use as ROS base file for pubs and subs
      ros::Publisher  joints_pub_;    // publisher to moveit fake joints controllers (for RViz visualization)
      ros::Publisher  trajectory_pub_;// trajectory publisher to /trajectory topic
      ros::Publisher  stop_pub_;      // publisher to stop trajectory execution 
      ros::Subscriber joints_sub_;    // subscriber to joint current states
      ros::Subscriber trajpoint_sub_; // subscriber to the trajectory counter

    // Dynamic planner params definition
      DynamicPlannerParams params_;
      std::string planning_group_name_;                           // Planning group
      std::vector<std::string> joints_names_group_;               // Joints group names
      std::vector<double> joints_values_group_;                   // Joints values      
      std::unordered_map<std::string, double> joints_map_group_;  // Set group joints names and values through mapping
      bool dynamic_behaviour_;                                    // Set automatic check trajectory within the ROS spinner

      // Set joints limit through mapping the name of the configuration with a user defined 
      // list of min-max values on pos-vel-acc
      // While the desired limits are forced by user within the robot model for planning purposes, 
      // the following map structure holds the old values, so limits set with model initial loading are stored 
      std::unordered_map<std::string, moveit::core::VariableBounds> original_joints_limits_map_;

    // ROBOT MODEL AND STATE DEFINITION
    
      // Basic state and model import variables
      robot_model_loader::RobotModelLoader  robot_model_loader_;
      robot_model::RobotModelPtr            robot_model_ = nullptr;
      robot_state::RobotStatePtr            robot_state_ = nullptr;
      // Joint model groups
      const robot_model::JointModelGroup*   joint_model_group_;

    // PLANNING SCENE SETUP

      // Planning scene
      std::vector<moveit_msgs::CollisionObject>           collision_objects_;         // collision objects within the planning scene
      std::vector<moveit_msgs::AttachedCollisionObject>   attached_objects_;          // attached (to the robot) objects within the planning scene
      planning_scene::PlanningScenePtr                    planning_scene_ = nullptr;  // MoveIt! planning scene
      moveit_visual_tools::MoveItVisualToolsPtr           visual_tools_   = nullptr;  // MoveIt! visual tools
      moveit::planning_interface::PlanningSceneInterface  planning_scene_interface_;  // MoveIt! planning scene interface

      // Planning pipeline client setup
      planning_pipeline::PlanningPipelinePtr planning_pipeline_ = nullptr;
      planning_interface::MotionPlanRequest  request_;                      // Client request  to MoveIt
      planning_interface::MotionPlanResponse result_;                       // Client result from MoveIt

      // Planning names
      PlanningSpace planning_space_;    // MoveIt! planning space object, parent class of useful functions
      std::string planning_link_name_;  // the name of the link as parent frame of the goal pose
      std::string planning_group_;      // name of the group of joints to be controlled
      std::string ee_link_name_;        // Gripper link name

      // Planning outputs
      moveit_msgs::Constraints      goal_;              // Moevit msgs about goal constraints 
      std::vector<double>           final_position_;    // Final joints position
      geometry_msgs::PoseStamped    final_pose_;        // Final TCP 3D position
      moveit_msgs::RobotTrajectory  trajectory_;        // Moevit msgs about trajectory
      ulong trajpoint_;                                 // Current point of the trajectory considered
      std::vector<moveit_msgs::Constraints> goals_seq_; // Goals sequence given to the planner

      ulong invalid_state_;           // point of the trajectory where a collision has been found
      bool success_;                  // whereas trajectory planning has been successfull
      bool joints_group_received_;    // Check if joints group has been received from the planner
      bool obstruction_;              // wheather an obstacle is on the robot path (1) or not (0)
      bool sim_;                      // Simulation status: 1 for simulation, 0 for debug

  // ---------------------  PRIVATE FUNCTIONS ------------------
    // Initialization of the planner, the node and other params
    void initialize(const double v_factor, const double a_factor);

    // Joints limit handling
    void adaptJointsLimits();           // enlarge joints limits
    void restoreOriginalJointsLimits(); // restore original joints limits as loaded by robot model

    // Merge a user defined trajectory with the running one depending on their relative length
    void merge(moveit_msgs::RobotTrajectory& robot_trajectory);

    // MoveIt! planning to goal
    void plan(const moveit_msgs::Constraints& desired_goal, const bool send = true);
    // MoveIt! planning to many goals through trajctory
    void plan(const std::vector<moveit_msgs::Constraints>& desired_goals,
              moveit_msgs::RobotTrajectory& robot_trajectory);

    // Visualization function
    void trajectoryVisualizer(moveit_msgs::RobotTrajectory& robot_trajectory);

    // Check given goal is different from current pose
    const bool checkJointDiff(const std::vector<double>& final_position);

    // CALLBACK FUNCTIONS FOR SUBSCRIBERS
    // Trajectory points current state 
    void trajPointCallback(const std_msgs::Int32::ConstPtr& traj_point);
    // Joints current state
    void jointsCallback(const sensor_msgs::JointState::ConstPtr& joints_state);
};

#endif /* DYNAMIC_PLANNER_H */