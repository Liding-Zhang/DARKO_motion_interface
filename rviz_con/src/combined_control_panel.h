#ifndef COMBINED_CONTROL_PANEL_H
#define COMBINED_CONTROL_PANEL_H

#include <QComboBox>
#include <QPushButton>
#include <QSlider>
#include <rviz/panel.h>

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h> // For MoveIt 1
#include <moveit_msgs/MoveGroupAction.h>
#include <pthread.h>
#include <qlineedit.h>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

namespace my_rviz_panel {

class CombinedControlPanel : public rviz::Panel {
    Q_OBJECT

public:
    CombinedControlPanel(QWidget* parent = 0);
    ros::NodeHandle nh_;
    // PARAMETERS
    std::string PREFIX = "";
    std::string baseFrameId = "base_footprint";
    std::string mapFrameId = "robot_map";
    std::string planningGroupName = "panda_arm";

    // Functions
    sensor_msgs::JointState getTargetJointState();

    // Fields
    std::vector<QLineEdit*> baseCoordinates;

    // ROS Communication
    actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction>* move_group_client_;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>* move_base_client_;
    tf::TransformBroadcaster tf_broadcaster_;

signals:
    void planningResult(
        bool success, const moveit::planning_interface::MoveGroupInterface::Plan& plan);

private Q_SLOTS:
    void publishJointGoal();
    void publishBaseGoal();
    void publishAllGoals();

private:
    // UI Elements
    std::vector<QSlider*> jointSliders;
    QPushButton* jointGoalButton;
    QPushButton* baseGoalButton;
    QPushButton* publishAllButton;
    QComboBox* armPlannerComboBox;
    QComboBox* basePlannerComboBox;

    // Configuration
    std::vector<std::string> jointNames = { "panda_joint1", "panda_joint2", "panda_joint3",
                                            "panda_joint4", "panda_joint5", "panda_joint6",
                                            "panda_joint7" };

    // Planning Scene Monitor for MoveIt 1
    std::unique_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;

    // Add a thread to handle planning
    pthread_t plannerThread;
    pthread_mutex_t joint_mutex;
    pthread_mutex_t base_mutex;

    // Function to start planning in the background thread
    static void* planJointGoalThread(void* arg);
    // Function to start planning in the background thread
    static void* planBaseGoalThread(void* arg);
    // Function to load node params
    void loadParameters();

    void loadAvailableBasePlanners();
    void loadAvailableArmPlanners();
};

} // namespace my_rviz_panel

#endif // COMBINED_CONTROL_PANEL_H
