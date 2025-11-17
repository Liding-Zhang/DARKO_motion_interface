# Combined RViz Interface for Mobile Manipulators
This repository contains packages for running a motion planning simulation for a Darko base robot, combined with a Franka Emika manipulator.
More specifically, this repo also contains a custom catkin package (`rviz_con`), which allows the motion planning of the two robots of a mobile manipulator 
to be combined/executed simultaneously (!improvement needed!), using an RViz Panel.

## Build instructions
Copy everything in the `src` folder and run:
```
catkin build
```
If you only want to build the `rviz_con` package separately (e.g., after changing code in it), run:
```
catkin build rviz_con
```
**! Important Note !**:  
Before building, you might need to adjust the export path at the bottom of `rviz_con/package.xml`, such that it is a valid absolute path pointing to `rviz_con/rviz_plugin_description_files/combined_control_panel.xml`.

## Run instructions
These are the instructions to start the simulation of the mobile manipulator, followed by the combined interface.
First, source the `devel` folder:
```
source devel/setup.bash
```
Start the combined simulation:
```
roslaunch rbkairos_sim_bringup rbkairos_franka.launch moveit_movegroup_a:=true
```
After this, run:
```
roslaunch rviz_con rviz.launch prefix:="robot/"
```
It is important to run the combined interface after the simulation has started, as `rviz_con` tries to connect with the `move_group` and `move_base` topics.
If they are in a namespace, like in this case, you need to specify it using the `prefix` argument of the launch file.

If the `roslaunch` command above did not work somehow, you can start the Panel manually:
```
rosrun rviz rviz
```
Then, on RViz, select `Panels` -> `CombinedControlPanel`.
If you started manually, you can save the state of the RViz instance to be launched by the launch file by selecting `File` -> `Save Config As` and then saving it to: `rviz_con/config/rviz_con.rviz`

## TODOs
Here, we have listed some issues/ideas that remain to be solved/implemented.

### Issues:
- [1] **Moving at the same time**: Currently, the "Publish All" button starts the base and manipulator planning in two separate threads. However, as the base is much faster in planning, it starts to move before the manipulator does. Joining the threads to one to move them together would be good. One could also adjust the movement speed of the base such that they finish simultaneously.
- [2] **"Failed" execution**: Even though the manipulator's movement finishes, the execution somehow fails. This should be avoided to distinguish real failures from normal executions.

### Ideas:
- [ ] **Robot-Independence**: Currently, the `rviz_con` package only allows for this specific combination of mobile manipulators, but it would be interesting to allow dynamic combinations, as it would open the door to combined planning for all mobile manipulators.
- [ ] **True combined motion planning**: Currently, the failure of one motion thread does not affect the other. We can implement a system where threads send each other interrupt signals to communicate their needs.
- [ ] **Visualize planning goal**: You could visualize the planning goal / the joint positions of the manipulator in the RViz instance of `rviz_con`.
- [ ] **ROS topic as an API**: Implementing a ROS topic for communicating the goal between programs would be great, as currently `rviz_con` only allows goal specifications via user input.

