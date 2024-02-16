# dynamic_planner

This repository contains an automatic dynamic_planner library based on MoveIt Framework.

This library is the main component of the work presented in: 

*A. Pupa, M. Arrfou, G. Andreoni and C. Secchi, "A Safety-Aware Kinodynamic Architecture for Human-Robot Collaboration," in IEEE Robotics and Automation Letters, vol. 6, no. 3, pp. 4465-4471, July 2021, doi: [10.1109/LRA.2021.3068634](https://doi.org/10.1109/LRA.2021.3068634) .*

and it has been developed during the [**ROSSINI**](https://www.rossini-project.com/) European project.

## Getting Started

### Prerequisites 

Go inside the workspace folder and install all the required packages
```
rosdep install --from-paths src --ignore-src -y
```
Compile the node
```
catkin_make
```

## How to use
The library can be called from any other node.
To do this you have to modify the CMakeLists.txt
```
find_package(catkin REQUIRED COMPONENTS
  ...
  dynamic_planner
  ...
)
```
and the package.xml
```xml
<depend>dynamic_planner</depend>
```
At this point inside your code you have to include the header file:
```c
#include "dynamic_planner/dynamic_planner.h"
```
And you can simply use inside your node:
```c
int main()
  ...

  //Istantiate class
  DynamicPlanner DynamicPlanner(manipulator_name, joint_names, vel_factor, acc_factor);
  
  //Create actual joint goal
  std::vector<double> q(6,0.0);
  planner_->plan(q);
 
  ...
```

where *manipulator_name* is tha name of the the moveit_planning group, *joint_names* are the name of the joints of the urdf, *vel_factor* and *acc_factor* represent the velocity and acceleration scaling factor (from 0 to 1).

After you create your new node that calls the DynamicPlanner, e.g. manipulator_planner, you have to launch it as:

```
...

<arg name="planning_plugin" value="ompl_interface/OMPLPlanner" />
  <arg name="planning_adapters" value="industrial_trajectory_filters/UniformSampleFilter default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints" />

  <node name="manipulator_planner" pkg="manipulator_planner" type="manipulator_planner" output="screen">
    <param name="planning_plugin" value="$(arg planning_plugin)" />
    <param name="request_adapters" value="$(arg planning_adapters)" />
    <param name="sample_duration" value="0.01"/>
  </node>

  ...
```

**NOTE:** Remember that you have also to launche the moveit packages of your robot (planning_context, ...)!

## Authors
* **Andrea Pupa** 
* **Simone Comari** 

## Maintainer
* **Andrea Pupa** 
* **Simone Comari** 
* **Italo Almirante** 

# Issue Statement

## Reporting Issues

If you encounter any difficulties compiling the nodes or experience issues while using this repository, we encourage you to report them. You can do so by either [opening an issue](https://github.com/apupa/dynamic_planner/issues) directly on GitHub or by sending an email to [andrea.pupa@unimore.it](andrea.pupa@unimore.it).

## Important Note

This repository is currently in a preliminary version, and we are actively working on it during our spare time. As such, you may encounter bugs or incomplete features. We appreciate your understanding and patience as we continue to improve and refine the codebase.

Thank you for your interest and contributions!