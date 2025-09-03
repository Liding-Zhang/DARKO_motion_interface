
# Combined control Rviz panel
This package provides you with a Rviz panel for a combined interface for planning the motion of the rbkairos mobile manipulator.
How to build and run this package is described in the base [README.md](../../README.md) file.

## What it does 
This is a broad description of how this program behaves:
- Read in arguments specified by the user (see `loadParameters()`).
- Tries to connect to `move_base` and `move_group` topic using `actionlib::SimpleActionClient`.
- User can specify goals via GUI.
- If a button is pressed execute the specified planning (see `combined_control_panel.cpp:61-63`) using threads.

## Summary of `CMakeLists.txt`
This section summarizes the specifications of the `CMakeLists.txt` file.

### **Dependencies**
- Requires ROS components: `tf`, `rviz`, `moveit_ros_planning_interface`, `moveit_core`, `moveit_kinematics`, `actionlib`, `move_base_msgs`, `tf2_geometry_msgs`.
- Requires `Qt5` Widgets module.

### **Build & Linking**
- Builds the `rviz_con` library from `combined_control_panel.cpp` and `combined_control_panel.h` (with Qt MOC).
- Links against `catkin`, `Qt5`, `rviz`, and `MoveIt!` libraries.

### **Installation**
- Installs the `rviz_con` library to standard ROS locations.
- Installs the `rviz_plugin_description_files` directory. 

### Message, Service, and Action Generation
Sections for generating ROS messages, services, and actions are commented out, but they provide instructions on how to configure message/service/action generation if needed.
