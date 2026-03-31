/**
 * arm_commander_node.cpp
 *
 * C++ port of the Python ArmCommander node.
 * Subscribes to:
 *   /arm_command/pose_goal  (geometry_msgs/PoseStamped)  → Cartesian goal
 *   /arm_command/joint_goal (sensor_msgs/JointState)     → Joint-space goal
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <atomic>
#include <map>
#include <string>

class ArmCommander : public rclcpp::Node
{
public:
  explicit ArmCommander(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("arm_commander_node", options),
    is_moving_(false)
  {
    RCLCPP_INFO(get_logger(), "Starting ArmCommander node...");
    // MoveGroupInterface is constructed in init() so that shared_from_this()
    // is already valid at that point.
  }

  // Call once after construction (see main()).
  void init()
  {
    arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), "arm");

    arm_->setPlanningTime(10.0);
    arm_->setNumPlanningAttempts(10);

    // Cartesian-goal subscriber
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/arm_command/pose_goal",
      rclcpp::QoS(10),
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
        pose_callback(msg);
      });

    // Joint-space-goal subscriber
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/arm_command/joint_goal",
      rclcpp::QoS(10),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
        joint_callback(msg);
      });

    RCLCPP_INFO(get_logger(), "Commander ready! Listening on /arm_command/...");
  }

private:
  // Callbacks 

  void pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr & msg)
  {
    // exchange(true) returns the OLD value; if it was already true, we're busy
    if (is_moving_.exchange(true)) {
      RCLCPP_WARN(get_logger(), "Arm is already moving. Ignoring command.");
      return;
    }

    RCLCPP_INFO(get_logger(),
      "Cartesian goal received: X=%.3f  Y=%.3f  Z=%.3f",
      msg->pose.position.x,
      msg->pose.position.y,
      msg->pose.position.z);

    arm_->setPoseTarget(msg->pose, "link_6");
    plan_and_execute();
  }

  void joint_callback(const sensor_msgs::msg::JointState::ConstSharedPtr & msg)
  {
    if (is_moving_.exchange(true)) {
      RCLCPP_WARN(get_logger(), "Arm is already moving. Ignoring command.");
      return;
    }

    if (msg->name.size() != msg->position.size()) {
      RCLCPP_ERROR(get_logger(),
        "JointState name/position size mismatch (%zu vs %zu). Aborting.",
        msg->name.size(), msg->position.size());
      is_moving_.store(false);
      return;
    }

    RCLCPP_INFO(get_logger(), "Joint-space goal received.");

    // Build { joint_name → angle_rad } map and pass it to MoveIt
    std::map<std::string, double> joint_goal;
    for (std::size_t i = 0; i < msg->name.size(); ++i)
      joint_goal[msg->name[i]] = msg->position[i];

    arm_->setJointValueTarget(joint_goal);
    plan_and_execute();
  }

  // Core planner / executor

  void plan_and_execute()
  {
    RCLCPP_INFO(get_logger(), "Computing collision-free trajectory...");

    moveit::planning_interface::MoveGroupInterface::Plan safe_plan;
    const bool planned =
      (arm_->plan(safe_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (planned) {
      RCLCPP_INFO(get_logger(), "Collision-free path found! Executing...");

      const bool executed =
        (arm_->execute(safe_plan) == moveit::core::MoveItErrorCode::SUCCESS);

      if (executed)
        RCLCPP_INFO(get_logger(), "Goal reached successfully.");
      else
        RCLCPP_ERROR(get_logger(), "Execution failed.");
    } else {
      RCLCPP_ERROR(get_logger(),
        "Motion aborted! Path collides or goal is unreachable.");
    }

    is_moving_.store(false);
  }

  // Members 
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;

  std::atomic<bool> is_moving_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ArmCommander>(
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  node->init();

  // MultiThreadedExecutor: MoveIt planning and topic callbacks run concurrently
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  try {
    executor.spin();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "Unhandled exception: %s", e.what());
  }

  rclcpp::shutdown();
  return 0;
}