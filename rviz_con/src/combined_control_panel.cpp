#include "combined_control_panel.h"
#include <QApplication>
#include <QLabel>
#include <QVBoxLayout>
#include <spdlog/spdlog.h>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/MoveGroupAction.h>
#include <nav_core/base_global_planner.h>
#include <pluginlib/class_loader.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace my_rviz_panel {

CombinedControlPanel::CombinedControlPanel(QWidget* parent)
    : rviz::Panel(parent)
    , tf_broadcaster_()
    , nh_("~")
{
    QVBoxLayout* layout = new QVBoxLayout;
    spdlog::info("Load params...");
    loadParameters();

    spdlog::info("Initializing Joint Sliders...");
    // Create joint sliders
    for (const auto& jointName : jointNames) {
        QSlider* slider = new QSlider(Qt::Horizontal);
        slider->setRange(-180, 180);
        jointSliders.push_back(slider);
        layout->addWidget(new QLabel(QString::fromStdString(jointName)));
        layout->addWidget(slider);
    }

    spdlog::info("Initializing Base Coordinates...");
    // Create base coordinate input fields (x, y, yaw)
    for (const auto& coordName : { "x", "y", "yaw" }) {
        QLineEdit* edit = new QLineEdit;
        baseCoordinates.push_back(edit);
        layout->addWidget(new QLabel(QString::fromStdString(coordName)));
        layout->addWidget(edit);
    }

    // Create buttons
    spdlog::info("Create Buttons...");
    jointGoalButton = new QPushButton("Publish Joint Goal");
    baseGoalButton = new QPushButton("Publish Base Goal");
    publishAllButton = new QPushButton("Publish All Goals");
    layout->addWidget(jointGoalButton);
    layout->addWidget(baseGoalButton);
    layout->addWidget(publishAllButton);

    // If a button is pressed, execute...
    connect(jointGoalButton, SIGNAL(clicked()), this, SLOT(publishJointGoal()));
    connect(baseGoalButton, SIGNAL(clicked()), this, SLOT(publishBaseGoal()));
    connect(publishAllButton, SIGNAL(clicked()), this, SLOT(publishAllGoals()));

    // Create planner combo boxes
    armPlannerComboBox = new QComboBox;
    basePlannerComboBox = new QComboBox;

    // Add some common planner options (you can customize this list)
    armPlannerComboBox->addItems({ "RRTConnect", "RRTstar", "PRMstar" });
    basePlannerComboBox->addItems({ "GlobalPlanner", "NavfnROS", "CarrotPlanner" });

    /* loadAvailableArmPlanners(); */

    // Add widget for choosing planner
    spdlog::info("Adding Planner Chooser...");
    layout->addWidget(new QLabel("Arm Planner:"));
    layout->addWidget(armPlannerComboBox);
    layout->addWidget(new QLabel("Base Planner:"));
    layout->addWidget(basePlannerComboBox);

    setLayout(layout);

    // ROS Communication Setup
    spdlog::info("Create Action Clients...");
    move_group_client_ = new actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction>(
        nh_, "/" + PREFIX + "move_group", true);
    move_base_client_ = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>(
        nh_, "/" + PREFIX + "move_base", true);

    // Set the timeout duration
    ros::Duration timeout(5.0); // 5 seconds

    spdlog::info("Trying to connect ...");

    // Wait for the servers with timeout
    if (!move_group_client_->waitForServer(timeout)) {
        spdlog::error("Failed to connect to move_group server within timeout!");
        // Handle the timeout (e.g., disable the joint goal button)
        jointGoalButton->setEnabled(false);
        publishAllButton->setEnabled(false);
    } else {
        spdlog::info("Connected to move_group server!");
    }

    if (!move_base_client_->waitForServer(timeout)) {
        spdlog::error("Failed to connect to move_base server within timeout!");
        // Handle the timeout (e.g., disable the base goal button)
        baseGoalButton->setEnabled(false);
        publishAllButton->setEnabled(false);
    } else {
        spdlog::info("Connected to move_base server!");
    }

    // Planning Scene Monitor for MoveIt 1
    spdlog::info("Initialize Planning Scene Monitor...");
    planning_scene_monitor_.reset(
        new planning_scene_monitor::PlanningSceneMonitor(PREFIX + "robot_description"));
    planning_scene_monitor_->startSceneMonitor();
    planning_scene_monitor_->startStateMonitor();
    planning_scene_monitor_->startWorldGeometryMonitor();

    // Initialize the mutex
    pthread_mutex_init(&joint_mutex, NULL);
    pthread_mutex_init(&base_mutex, NULL);
}

void CombinedControlPanel::publishJointGoal()
{
    if (!move_group_client_) {
        spdlog::error("MoveGroup action client not initialized!");
        return;
    }

    // Create and start the thread
    pthread_create(&plannerThread, NULL, &CombinedControlPanel::planJointGoalThread, this);
}

void* CombinedControlPanel::planJointGoalThread(void* arg)
{
    CombinedControlPanel* panel = static_cast<CombinedControlPanel*>(arg);

    // Create a MoveGroupGoal message
    moveit_msgs::MoveGroupGoal goal;
    goal.request.group_name = panel->planningGroupName;
    goal.request.allowed_planning_time = 1;
    goal.request.num_planning_attempts = 5; // You can adjust this if needed
    goal.request.planner_id = panel->armPlannerComboBox->currentText().toStdString();
    spdlog::info("Using arm planner: {}", goal.request.planner_id);

    // Fill in the planning request
    goal.planning_options.plan_only = true;
    goal.planning_options.planning_scene_diff.is_diff = true; // Assuming no scene changes
    goal.planning_options.planning_scene_diff.robot_state.is_diff = true;

    // Set the target joint values in the goal message
    goal.request.goal_constraints.resize(1);
    moveit_msgs::JointConstraint joint_constraint;
    for (size_t i = 0; i < panel->jointNames.size(); ++i) {
        joint_constraint.joint_name = panel->jointNames[i];
        joint_constraint.position =
            panel->jointSliders[i]->value() * M_PI / 180.0; // Convert to radians
        joint_constraint.tolerance_above = 0.01; // Add some tolerance (adjust as needed)
        joint_constraint.tolerance_below = 0.01;
        joint_constraint.weight = 1.0;
        goal.request.goal_constraints[0].joint_constraints.push_back(joint_constraint);
    }

    // Send the goal and wait for a result
    panel->move_group_client_->sendGoal(goal);
    bool success = panel->move_group_client_->waitForResult(); // Set a timeout

    // Check the result
    if (success) {
        moveit_msgs::MoveGroupResultConstPtr result = panel->move_group_client_->getResult();

        if (result->error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
            spdlog::info("Planning successful! Now executing...");
            // Reuse the goal message for execution
            goal.planning_options.plan_only = false;
            panel->move_group_client_->sendGoal(goal);
            success = panel->move_group_client_->waitForResult(
                ros::Duration(10.0)); // Set a timeout for execution

            if (success) {
                // Execution succeeded
                spdlog::info("Execution successful!");
            } else {
                // Execution failed or timed out
                spdlog::warn("Execution failed or timed out!");
            }
        } else {
            // Planning failed
            spdlog::warn("Planning failed with error code: {}", result->error_code.val);
        }
    } else {
        // Action timed out
        spdlog::warn("Planning action timed out!");
    }

    // You can emit a signal here to inform the GUI thread about the planning result.
    return NULL;
}

void CombinedControlPanel::publishBaseGoal()
{
    if (!move_base_client_) {
        spdlog::error("MoveGroup action client not initialized!");
        return;
    }
    pthread_create(&plannerThread, NULL, &CombinedControlPanel::planBaseGoalThread, this);
}

void* CombinedControlPanel::planBaseGoalThread(void* arg)
{
    CombinedControlPanel* panel = static_cast<CombinedControlPanel*>(arg);
    ROS_INFO_STREAM("Initializing goal...");
    geometry_msgs::PoseStamped basePose;
    basePose.header.frame_id = panel->mapFrameId;
    basePose.header.stamp = ros::Time::now();
    basePose.pose.position.x = panel->baseCoordinates[0]->text().toDouble();
    basePose.pose.position.y = panel->baseCoordinates[1]->text().toDouble();
    basePose.pose.orientation =
        tf::createQuaternionMsgFromYaw(panel->baseCoordinates[2]->text().toDouble() * M_PI / 180.0);

    panel->nh_.setParam(
        "/" + panel->PREFIX + "move_base/base_global_planner",
        panel->basePlannerComboBox->currentText().toStdString());
    spdlog::info("Using base planner: {}", panel->basePlannerComboBox->currentText().toStdString());

    // lock mutex
    pthread_mutex_lock(&(panel->base_mutex));
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose = basePose;
    ROS_INFO_STREAM("Send goal...");
    panel->move_base_client_->sendGoal(goal);
    ROS_INFO_STREAM("Waiting for result...");
    bool success = panel->move_base_client_->waitForResult();
    ROS_INFO_STREAM("Execution: " << success ? "Success!" : "Failed!");

    // Broadcast transform (for ROS1 using tf)
    /* ROS_INFO_STREAM("Initializing transformation..."); */
    /* tf::Transform transform; */
    /* transform.setOrigin(tf::Vector3(basePose.pose.position.x, basePose.pose.position.y, 0.0)); */
    /* transform.setRotation(tf::Quaternion(basePose.pose.orientation.x,
     * basePose.pose.orientation.y, basePose.pose.orientation.z, basePose.pose.orientation.w)); */

    /* ROS_INFO_STREAM("Sending transformation..."); */
    /* panel->tf_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
     * panel->mapFrameId, panel->baseFrameId)); */

    pthread_mutex_unlock(&(panel->base_mutex));
    ROS_INFO_STREAM("Base Goal: Done!");
    return NULL;
}

void CombinedControlPanel::publishAllGoals()
{
    if (!move_base_client_) {
        spdlog::error("MoveGroup action client not initialized!");
        return;
    }
    if (!move_group_client_) {
        spdlog::error("MoveGroup action client not initialized!");
        return;
    }

    // Create and start the thread
    pthread_create(&plannerThread, NULL, &CombinedControlPanel::planJointGoalThread, this);
    pthread_create(&plannerThread, NULL, &CombinedControlPanel::planBaseGoalThread, this);
}

sensor_msgs::JointState CombinedControlPanel::getTargetJointState()
{
    sensor_msgs::JointState jointState;
    jointState.name = jointNames;
    jointState.position.resize(jointNames.size());
    for (size_t i = 0; i < jointNames.size(); ++i) {
        jointState.position[i] = jointSliders[i]->value() * M_PI / 180.0;
    }
    return jointState;
}

void CombinedControlPanel::loadParameters()
{
    // Read the prefix parameter (using the private namespace ~)
    if (!nh_.getParam("prefix", PREFIX)) {
        ROS_WARN_STREAM("Prefix parameter not found. Using default empty prefix.");
        PREFIX = ""; // Default value
    } else {
        ROS_INFO_STREAM("Using prefix: " << PREFIX);
    }
    if (!nh_.getParam("baseFrameId", baseFrameId)) {
        ROS_WARN_STREAM("Base frame parameter not found. Using default.");
        PREFIX = ""; // Default value
    } else {
        ROS_INFO_STREAM("Using base frame ID: " << baseFrameId);
    }
    if (!nh_.getParam("mapFrameId", mapFrameId)) {
        ROS_WARN_STREAM("Map frame parameter not found. Using default.");
        PREFIX = ""; // Default value
    } else {
        ROS_INFO_STREAM("Using map frame ID: " << mapFrameId);
    }
    if (!nh_.getParam("pandaGroupName", planningGroupName)) {
        ROS_WARN_STREAM("Group name parameter not found. Using default.");
        PREFIX = ""; // Default value
    } else {
        ROS_INFO_STREAM("Using Panda planning group name: " << planningGroupName);
    }
}

void CombinedControlPanel::loadAvailableBasePlanners()
{
    // Create a PluginLoader for BaseGlobalPlanner plugins
    pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> base_planner_loader(
        "nav_core", "nav_core::BaseGlobalPlanner");

    // Get the names of all loaded plugins
    std::vector<std::string> planner_names = base_planner_loader.getDeclaredClasses();

    // Add planner names to the combo box
    for (const std::string& planner_name : planner_names) {
        basePlannerComboBox->addItem(QString::fromStdString(planner_name));
    }

    if (!planner_names.empty()) {
        // Set the default planner to the first available one
        basePlannerComboBox->setCurrentIndex(0);
    } else {
        spdlog::warn("No base planners found.");
        basePlannerComboBox->addItem(QString::fromStdString("None"));
        baseGoalButton->setEnabled(false); // Disable button if no planners
    }
}

void CombinedControlPanel::loadAvailableArmPlanners()
{
    moveit::planning_interface::MoveGroupInterface move_group(planningGroupName);
    planning_pipeline::PlanningPipeline planning_pipeline(move_group.getRobotModel(), nh_);

    // Get the names of available planner plugins
    ros::V_string a;
    planning_pipeline.getPlannerManager().get()->getPlanningAlgorithms(a);
    for (std::string s : a) {
        spdlog::info(s);
    }
    /* for (const std::string &planner_id : planner_ids) */
    /* { */
    /*     armPlannerComboBox->addItem(QString::fromStdString(planner_id)); */
    /* } */

    /* if (!planner_ids.empty()) */
    /* { */
    /*     // Set the default planner to the first available one */
    /*     armPlannerComboBox->setCurrentIndex(0); */
    /* } */
}

} // namespace my_rviz_panel

// Pluginlib Macro
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(my_rviz_panel::CombinedControlPanel, rviz::Panel)
