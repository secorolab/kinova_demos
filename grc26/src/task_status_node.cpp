#include "grc26/task_status_node.hpp"

TaskStatusROSNode::TaskStatusROSNode(std::shared_ptr<TaskStatus> task_status)
    : Node("task_status_node"),
      task_status_(task_status)
{
    rclcpp::QoS qos(1);
    qos.transient_local();
    qos.reliable();

    status_pub_ = create_publisher<grc26::msg::TaskStatus>(
        "/task_status", qos);

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
    grc26::msg::TaskStatus msg;

    msg.idle = status.idle;
    msg.human_initiation = status.human_initiation;
    msg.task_completion = status.task_completion;
    msg.obj_held_by_human = status.obj_held_by_human;

    status_pub_->publish(msg);
}