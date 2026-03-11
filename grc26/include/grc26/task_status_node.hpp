#ifndef TASK_STATUS_NODE_HPP
#define TASK_STATUS_NODE_HPP

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <grc26/msg/task_status.hpp>

#include "grc26/task_status.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "bdd_ros2_interfaces/msg/event.hpp"
#include "bdd_ros2_interfaces/msg/trinary_stamped.hpp"
#include "control_msgs/action/gripper_command.hpp"


using GripperCommand = control_msgs::action::GripperCommand;
using GoalHandleGripperCommand = rclcpp_action::ClientGoalHandle<GripperCommand>;

class TaskStatusROSNode : public rclcpp::Node
{
public:
    explicit TaskStatusROSNode(std::shared_ptr<TaskStatus> task_status);

private:
    void publishCallback();
    void publishStatus(TaskStatusData& status);
    int send_goal(const control_msgs::action::GripperCommand::Goal& goal_msg);

    std::shared_ptr<TaskStatus> task_status_;

    rclcpp::Publisher<bdd_ros2_interfaces::msg::Event>::SharedPtr event_pub_;

    rclcpp::Publisher<bdd_ros2_interfaces::msg::TrinaryStamped>::SharedPtr located_pick_pub_;
    rclcpp::Publisher<bdd_ros2_interfaces::msg::TrinaryStamped>::SharedPtr is_held_pub_;
    rclcpp::Publisher<bdd_ros2_interfaces::msg::TrinaryStamped>::SharedPtr located_place_pub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    rclcpp_action::Client<control_msgs::action::GripperCommand>::SharedPtr gripper_cmd_goal_publisher;

    int publish_rate_hz_ = 20;
};

#endif // TASK_STATUS_NODE_HPP