# ur5sim
<!-- ABOUT THE PROJECT -->
## About the project
In this project, an object sorting scenario is being simulated on CoppeliaSim. ROS interface is incorporated for controlling an UR5 robot arm through CoppeliaSim remote API, planning joint trajectory through MoveIt package. OpenCV python library is used for differentiating objects and getting the relative position and orientation.  

<!-- Simulator -->
## Simulator
Coppeliasim in docker container
<!-- Components -->

## Components
1. Robots: UR5, suction gripper
2. Sensors:  RGBD camera
3. Package and library: 	<ul>
                          <li>OpenCV</li>
                          <li>MoveIt</li>
                          <li>CoppeliaSim remote API</li>
                          <li>UR5’s urdf  from universal_robot package</li>
                          </ul>

<!-- How to start -->
## How to start
After cloning this repo, remmber to rebuild and source your catkin workspace.
In CoppeliaSim, open the screne file  `objectsorting.ttt`.
Then start the simulation and in a terminal, launch `ur5sim.launch`
```sh
roslaunch  ur5sim ur5sim.launch 
 ```


