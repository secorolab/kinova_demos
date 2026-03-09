#ifndef ACTION_SERVER_NODE_HPP
#define ACTION_SERVER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "bdd_ros2_interfaces/action/behaviour.hpp"
#include "bdd_ros2_interfaces/msg/trinary.hpp"
#include "grc26/msg/task_status.hpp"
#include "grc26/task_status.hpp"

struct BehaviourState {
    unique_identifier_msgs::msg::UUID bhv_ctx_id;
    bool goal_in = false;
    bool goal_done = false;
};

class ActionServerNode : public rclcpp::Node
{
public:
    ActionServerNode(std::shared_ptr<TaskStatus> task_status);

private:
    using Behaviour = bdd_ros2_interfaces::action::Behaviour;
    using GoalHandleBehaviour = rclcpp_action::ServerGoalHandle<Behaviour>;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const Behaviour::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleBehaviour> goal_handle);

    void handle_accepted(const std::shared_ptr<GoalHandleBehaviour> goal_handle);
    void execute(const std::shared_ptr<GoalHandleBehaviour> goal_handle);

    rclcpp_action::Server<Behaviour>::SharedPtr action_server_;
    std::shared_ptr<TaskStatus> task_status_;
};

#endif // ACTION_SERVER_NODE_HPP