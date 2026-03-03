// Copyright (c) 2023 Dexory

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/client.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"

using std::placeholders::_1;

class TestPlannerWithRviz : public rclcpp::Node
{
public:
  TestPlannerWithRviz()
  : Node("test_planner_with_rviz")
  {
    goal.use_start = true;

    subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "goal_pose", 10, std::bind(&TestPlannerWithRviz::goal_pose_callback, this, _1));

    callback_group_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive,
      false);
    callback_group_executor_.add_callback_group(callback_group_, this->get_node_base_interface());

    action_client_ = rclcpp_action::create_client<nav2_msgs::action::ComputePathToPose>(
      this, "compute_path_to_pose");
  }

private:
  void goal_pose_callback(const geometry_msgs::msg::PoseStamped & msg)
  {
    geometry_msgs::msg::PoseStamped pose_from_rviz = msg;
    if (!start_filled) {
      goal.start = pose_from_rviz;
      start_filled = true;
    } else {
      goal.goal = pose_from_rviz;
      RCLCPP_INFO(get_logger(), "Sending goal from: (%.2f, %.2f) [%.2f, %.2f, %.2f, %.2f] to (%.2f, %.2f) [%.2f, %.2f, %.2f, %.2f]",
        goal.start.pose.position.x, goal.start.pose.position.y,
        goal.start.pose.orientation.x, goal.start.pose.orientation.y, goal.start.pose.orientation.z, goal.start.pose.orientation.w,
        goal.goal.pose.position.x, goal.goal.pose.position.y,
        goal.goal.pose.orientation.x, goal.goal.pose.orientation.y, goal.goal.pose.orientation.z, goal.goal.pose.orientation.w);

      RCLCPP_INFO(get_logger(), "ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose \"{goal: {header: {frame_id: map}, pose: {position: {x: %.2f, y: %.2f, z: %.2f}, orientation: {x: %.2f, y: %.2f, z: %.2f, w: %.2f}}}, start: {header: {frame_id: map}, pose: {position: {x: %.2f, y: %.2f, z: %.2f}, orientation: {x: %.2f, y: %.2f, z: %.2f, w: %.2f}}}, use_start: true}\"",
        goal.goal.pose.position.x, goal.goal.pose.position.y, goal.goal.pose.position.z,
        goal.goal.pose.orientation.x, goal.goal.pose.orientation.y, goal.goal.pose.orientation.z, goal.goal.pose.orientation.w,
        goal.start.pose.position.x, goal.start.pose.position.y, goal.start.pose.position.z,
        goal.start.pose.orientation.x, goal.start.pose.orientation.y, goal.start.pose.orientation.z, goal.start.pose.orientation.w);

      RCLCPP_INFO(get_logger(), "START: Pose(position=Point(x=%.4f, y=%.4f, z=%.4f), orientation=Quaternion(x=%.4f, y=%.4f, z=%.4f, w=%.4f))",
        goal.start.pose.position.x, goal.start.pose.position.y, goal.start.pose.position.z,
        goal.start.pose.orientation.x, goal.start.pose.orientation.y, goal.start.pose.orientation.z, goal.start.pose.orientation.w);

      RCLCPP_INFO(get_logger(), "GOAL: Pose(position=Point(x=%.4f, y=%.4f, z=%.4f), orientation=Quaternion(x=%.4f, y=%.4f, z=%.4f, w=%.4f))",
        goal.goal.pose.position.x, goal.goal.pose.position.y, goal.goal.pose.position.z,
        goal.goal.pose.orientation.x, goal.goal.pose.orientation.y, goal.goal.pose.orientation.z, goal.goal.pose.orientation.w);

      action_client_->async_send_goal(goal);
      start_filled = false;
    }
  }
  nav2_msgs::action::ComputePathToPose::Goal goal;
  bool start_filled = false;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
  std::shared_ptr<rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>> action_client_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestPlannerWithRviz>());
  rclcpp::shutdown();
  return 0;
}
