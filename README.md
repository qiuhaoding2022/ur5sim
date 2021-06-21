# ur5sim
<!-- ABOUT THE PROJECT -->
In this project, an object sorting scenario is being simulated on CoppeliaSim. ROS interface is incorporated for controlling an UR5 robot arm through CoppeliaSim remote API, planning joint trajectory through MoveIt package. OpenCV python library is used for differentiating objects and getting the relative position and orientation.  

<!-- Simulator -->
Coppeliasim in docker container
<!-- Components -->
Robots: UR5, suction gripper
Sensors:  RGBD camera
Package and library: 	OpenCV
                      MoveIt
                      CoppeliaSim remote API
                      UR5’s urdf  from universal_robot package


<!-- How to start -->
After cloning this repo, remmber to rebuild and source your catkin workspace.
In CoppeliaSim, open the screne file  `objectsorting.ttt`.
roslaunch  ur5sim ur5sim.launch 



