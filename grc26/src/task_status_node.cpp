#include "grc26/task_status_node.hpp"

TaskStatusROSNode::TaskStatusROSNode(std::shared_ptr<TaskStatus> task_status)
    : Node("task_status_node"),
      task_status_(task_status)
{
    rclcpp::QoS qos(1);
    qos.transient_local();
    qos.reliable();

    located_pick_pub_ = create_publisher<bdd_ros2_interfaces::msg::TrinaryStamped>(
        "/obs_policy/located_at_pick_ws", qos);
    is_held_pub_ = create_publisher<bdd_ros2_interfaces::msg::TrinaryStamped>(
        "/obs_policy/is_held_ws", qos);
    located_place_pub_ = create_publisher<bdd_ros2_interfaces::msg::TrinaryStamped>(
        "/obs_policy/located_at_place_ws", qos);

    publish_timer_ = create_wall_timer(
        std::chrono::milliseconds(
            static_cast<int>(1000.0 / publish_rate_hz_)),
        std::bind(&TaskStatusROSNode::publishCallback, this));
}

void TaskStatusROSNode::publishCallback()
{
    TaskStatusData status;
    task_status_->getLatest(status);
    publishStatus(status);
}

void TaskStatusROSNode::publishStatus(const TaskStatusData& status)
{
    bdd_ros2_interfaces::msg::TrinaryStamped pick_msg;
    pick_msg.stamp = this->now();
    pick_msg.scenario_context_id = status.bhv_ctx_id;
    pick_msg.trinary.value = status.is_obj_located_at_pick_location ? bdd_ros2_interfaces::msg::Trinary::TRUE :
                             bdd_ros2_interfaces::msg::Trinary::FALSE;

    bdd_ros2_interfaces::msg::TrinaryStamped is_held_msg;
    is_held_msg.stamp = this->now();
    is_held_msg.scenario_context_id = status.bhv_ctx_id;
    is_held_msg.trinary.value = status.is_obj_held_by_robot ? bdd_ros2_interfaces::msg::Trinary::TRUE :
                                  bdd_ros2_interfaces::msg::Trinary::FALSE;

    bdd_ros2_interfaces::msg::TrinaryStamped place_msg;
    place_msg.stamp = this->now();
    place_msg.scenario_context_id = status.bhv_ctx_id;
    place_msg.trinary.value = status.is_obj_located_at_place_location ? bdd_ros2_interfaces::msg::Trinary::TRUE :
                               bdd_ros2_interfaces::msg::Trinary::FALSE;

    located_pick_pub_->publish(pick_msg);
    is_held_pub_->publish(is_held_msg);
    located_place_pub_->publish(place_msg);
}