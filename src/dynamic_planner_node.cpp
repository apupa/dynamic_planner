// IMPORT LIBRARY
#include "dynamic_planner/dynamic_planner.h"

// MAIN FUNCTION IMPLEMENTATION: this is a node
int main(int argc, char** argv)
{
	// Initiate the node
	ros::init(argc, argv, "dynamic_planner_node");

	// Start the asynchrounous ROS spinner (it handles time during ros execution)
	ros::AsyncSpinner spinner(1);
	spinner.start();

	// Set the manipulator name
	std::string a = "manipulator";
	
	// Set the joint names
	std::vector<std::string>  j = {"elbow_joint", "shoulder_lift_joint", "shoulder_pan_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
	// std::vector<std::string>  j = {"prbt_joint_1", "prbt_joint_2", "prbt_joint_3", "prbt_joint_4", "prbt_joint_5", "prbt_joint_6"};

	// What this vector corresponds to ?
	std::vector<double> i(6);
	i = {0.0,0.0,0.0,0.0,0.0,0.0};
	
	// Istantiate the object ce of class dynamic planner with its constructor
	DynamicPlanner ce(a,j,0.5,0.5);	// I HAVE CHANGED THIS LINE
	
	// Set/initialize the counter for the planning demo execution 
	int counter = 0;
	// Declare final position vector
	std::vector<double> final_position(6);
	// This variable is used for trajectory execution checking only
	int point_executed;

	// ROS LOOP FOR THE NODE -> NON HA MOLTO SENSO, ESEGUE SOLO UN MOVIMENTO, UTILE SOLO SE SI MODIFICA QUESTO STESSO FILE
	while(ros::ok())
	{	
		// Start the demo
		if(counter == 0){
			
			// Wait for the UI command
			ce.visual_tools_->prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

			// Set the final position of each joint
			final_position = {1.57, -1.57, 1.57, -1.57, -1.57, -1.57}; 

			// Update the counter
			counter = 1;

		} else if (counter == 1) {

			// Set the final position of each joint
			final_position = {0.7, -1.2, 1.57, -1.9, -1.57, -2.45};

			// Update the counter
			counter = 2;

		} else {

			// Set the final position of each joint
			final_position = {2.3, -1.2, 1.57, -1.9, -1.57, -2.45};

			// Update the counter
			counter = 1;

		}

		// Plan the trajectory to reach the joint final goal, but no execution !! (just for RViz visualization)
    	ce.plan(final_position);
		// ce.check_trajectory(ce.trajectory, point_executed); // Check if the trajectory has been executed
		ros::shutdown();	// Shutdown ROS when the move ends
		
		// ce.spinner();	// Spin the ROS loop

	}

// END OF THE CODE
return 0;

}
