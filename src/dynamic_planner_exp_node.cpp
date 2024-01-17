// IMPORT LIBRARY
#include "dynamic_planner/dynamic_planner.h"

// MAIN FUNCTION: this is a node
int main(int argc, char** argv)
{
	// Initiate the node
	ros::init(argc, argv, "dynamic_planner_exp_node");

	// Start the asynchrounous ROS spinner (it hendles time during ros execution)
	ros::AsyncSpinner spinner(1);
	spinner.start();

	// Set the manipulator name
	std::string a = "manipulator";

	// Set the joint names
	// std::vector<std::string>  j = {"elbow_joint", "shoulder_lift_joint", "shoulder_pan_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
	std::vector<std::string>  j = {"prbt_joint_1", "prbt_joint_2", "prbt_joint_3", "prbt_joint_4", "prbt_joint_5", "prbt_joint_6"};
	
	// ?????????????????' CHE CAZZO SONO QUESTI ?????????????????????????
	std::vector<double> l(6);
	l = {0.0,0.0,0.0,0.0,0.0,0.0};
	
	// Istantiate the object ce of class dynamic planner with its constructor
	DynamicPlanner ce(a,j,0.5,0.5);	// I HAVE CHANGED THIS LINE

	// Set/initialize the counter for the planning demo execution 
	int counter = 2;
	// Declare final position vector
	std::vector<double> final_position(6);

	// Create an obstacle and add it to the scene
	moveit_msgs::CollisionObject obstacle;

	// Set the frame to define the position of the obstacle relating to a parent
	obstacle.header.frame_id = "world";
    obstacle.id = "obstacle";

	// Set the shape of the obstacle as a SPHERE ????????????????
    obstacle.primitives.resize(1);
    obstacle.primitives[0].type = 2;
    obstacle.primitives[0].dimensions.push_back(0.06);

	// Set the position of the obstacle
    obstacle.primitive_poses.resize(1);
    obstacle.primitive_poses[0].position.x = -0.12;
    obstacle.primitive_poses[0].position.y = -0.2;
    obstacle.primitive_poses[0].position.z = 0.75;
    
	// Set the orientation of the obstacle
    obstacle.primitive_poses[0].orientation.x = 0;
    obstacle.primitive_poses[0].orientation.y = 0;
    obstacle.primitive_poses[0].orientation.z = 0;
    obstacle.primitive_poses[0].orientation.w = 1;

	// Set the obstacle as static ?????????????
    obstacle.operation = 0;

	// OBSTALCE ADDED TO THE LIST OF THE COLLISION OBJECT AND THEN TO THE SCENE
	// IS IT ACTUALLY THE RIGHT WAY TO DO IT? IN THE MANIPULATOR PLANNER IT IS DONE DIFFERENTLY
	// These commented lines are copied below

	// ce.collision_objects.push_back(obstacle);	
    // ce.planning_scene_interface.applyCollisionObjects(ce.collision_objects);

	// for(int i = 0; i < ce.collision_objects.size(); ++i)
    // {
    //     ce.planning_scene->processCollisionObjectMsg(ce.collision_objects[i]);
    // }


	// ROS LOOP EXECUTION
	while(ros::ok())
	{
		// Wait for the UI command
		ce.visual_tools_->prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

		// Execute a different move basing on the value of the counter set above (IT CAN BE CHANGE ONLY BY THIS CODE???????)
		if(counter == 0){

			// Set the final position of each joint
			final_position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

			// Update the counter
			counter = 1;

		} else if (counter == 1) {
			// Set the final position of each joint
			final_position =  {0.00, 0.0, 0.785, 1.57, 0.00, 0.0};
			
			// Update the counter
			counter = 2;
			
		} else {
			
			// Set the final position of each joint
			final_position =  {3.14, 0.0, 0.785, 1.57, 0.00, 0.0};
			
			// Update the counter
			counter = 1;

		}

		// Boolean variable to store the result of the trajectory execution
		bool done = false;

		// Plan the trajectory to reach the joint final goal, but no execution !!
    	ce.plan(final_position);
		ce.trajpoint_ = 0;

		// CHE CAZZO FA IN QUESTO LOOP ????

		// Until all the joints have not reached their goals 
		while(ce.trajpoint_ < ce.trajectory_.joint_trajectory.points.size()-1){
			
			// Spin ROS
			ce.spinner();

			// FIND THIS FUNCTION IN THE DYNAMIC PLANNER IMPLEMENTATION
			ce.checkTrajectory();

			// Until the demo is not finished, add an obstacle to the environment to see if check_trajectory function works
			if((ce.trajpoint_ > ce.trajectory_.joint_trajectory.points.size()*0.15) && (done == false) && counter == 2){
				ce.collision_objects_.push_back(obstacle);
				ce.planning_scene_interface_.applyCollisionObjects(ce.collision_objects_);

				for(int i = 0; i < ce.collision_objects_.size(); ++i)
				{
					ce.planning_scene_->processCollisionObjectMsg(ce.collision_objects_[i]);
				}
				done = true;
			}
		}

		// If the trajectory has been executed, shutdown the node
		// ros::shutdown();
		
		// Uncomment this line to go on with the ROS spinner
		// ce.spinner();

	}

// END OF THE CODE
return 0;

}
