## Phase 2 - "BVC Planner setup"
## Deliverable 1
Developing BVC node
- Install the following package to run the nonlinear optimization to generate controller setpoint for the robot
`sudo apt-get install ros-kinetic-ifopt`
- Follow installation instruction from the ![https://github.com/ethz-adrl/ifopt/tree/master](ifpopt repository)
- `project/include/problem_formulation.h` is an example nonlinear optimization rpoble used to generate separating hyperplanes of the voronoi cells that keep the robot safe during its motion.
## Deliverable 2
Hardware testing

## Phase 1 - "Simulation setup for the project"
### Deliverable 1
- This section of the project demonstrates launching multiple turtlebot3 robot instances from a single launch file, one of the robot will move with a collision avoidance algorithm and others act like dynamic obstacles. 
- In this submission simulation environment was setup and dynamic obstacles were moved with simple stop and go algorithm to wander around the space without any collision. 
- Following the below provided command open up the gazebo, simple move, and rviz. Simpl_move moves one turtlebot and the other one will be moved by the collision avoidance and the trajectory tracking in the next phases.
`
cd ros_ws
source devel/setup.bash
roslaunch project phase1.launch
`
### Deliverable 2
- The trajectory tracking node was not written from sratch because the aim of this project is to work on just a collision avoidance technique. Other sections of robot autonomy are taken from open source repositiories. 
- https://nagarjunvinukonda.github.io/files/Robot_Controls_Project_Final_Report.pdf was used to implement their trajectory tracking control.
- The following commands can be run on a terminal to launch the controller on the turtlebot simulation.
`
cd ros_ws
source devel/setup.bash
roslaunch project phase1_control.launch
`
### Deliberable 3
- `turtlebot3_slam` node is used to generate a map of the environment generated in teh simulation 1 with no dynamic obstacles.
- Following the below provided commands open up gazebo, teleoperation and slam nodes. By moving the turtlebot in circles around the world generates the ![map](/images/map.pgm) 
`
cd ros_ws
source devel/setup.bash
roslaunch project phase1_map.launch
`
In a new terminal, run a map_server node to save the generated map by the following commands
`
source devel/setup.bash
rosrun map_server map_saver -f map
`
Gmapping is compatible with the robots with Lidars and no cameras. Hence it can be implented for turtlebot3 robots. It can be installed with
`sudo apt-get install ros-noetic-slam-gmapping`