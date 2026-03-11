#include "grc26/task_status_node.hpp"

#include <string>

namespace {

const std::string kBddTemplateNamespace =
    "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/";

std::string makeTemplateEventUri(const char * event_name)
{
    return kBddTemplateNamespace + event_name;
}

}  // namespace


TaskStatusROSNode::TaskStatusROSNode(std::shared_ptr<TaskStatus> task_status)
    : Node("task_status_node"),
      task_status_(task_status)
{
    rclcpp::QoS qos(1);
    qos.transient_local();
    qos.reliable();

    event_pub_ = create_publisher<bdd_ros2_interfaces::msg::Event>(
        "/bdd/events", qos);

    located_pick_pub_ = create_publisher<bdd_ros2_interfaces::msg::TrinaryStamped>(
        "/obs_policy/located_at_pick_ws", qos);
    is_held_pub_ = create_publisher<bdd_ros2_interfaces::msg::TrinaryStamped>(
        "/obs_policy/is_held_ws", qos);
    located_place_pub_ = create_publisher<bdd_ros2_interfaces::msg::TrinaryStamped>(
        "/obs_policy/located_at_place_ws", qos);

    gripper_cmd_goal_publisher = rclcpp_action::create_client<control_msgs::action::GripperCommand>(this,
      "/robotiq_gripper_controller/gripper_cmd");

    printf("Waiting for gripper command action server...\n");   
    if (!gripper_cmd_goal_publisher->wait_for_action_server()) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        rclcpp::shutdown();
    }

    publish_timer_ = create_wall_timer(
        std::chrono::milliseconds(
            static_cast<int>(1000.0 / publish_rate_hz_)),
        std::bind(&TaskStatusROSNode::publishCallback, this));
}

void TaskStatusROSNode::publishCallback()
{
    TaskStatusData status;
    task_status_->consumeLatest(status);
    publishStatus(status);
}

void TaskStatusROSNode::publishStatus(TaskStatusData& status)
{
    if (status.gripper_send_goal){
        printf("--------------------->Sending gripper command goal with position: %f\n", status.desired_gripper_pos);
        status.gripper_send_goal = false;
        control_msgs::action::GripperCommand::Goal goal_msg;
        // std::array<std::string, 1> joint_names = {"robotiq_85_left_knuckle_joint"};
        // std::array<double, 1> positions = {status.desired_gripper_pos};
        // goal_msg.command.name = joint_names;
        goal_msg.command.position = status.desired_gripper_pos;
        auto result = send_goal(goal_msg);
        if (result) {
            RCLCPP_INFO(get_logger(), "Sent gripper command goal");
            status.gripper_send_goal = false;
            status.gripper_goal_sent = true;
        } else {
            RCLCPP_ERROR(get_logger(), "Failed to send gripper command goal. Retrying...");
            status.gripper_send_goal = true; // retry sending in the next cycle
        }
    }

    if (status.is_obj_located_at_pick_location) {
        bdd_ros2_interfaces::msg::TrinaryStamped msg;
        msg.stamp = this->now();
        msg.scenario_context_id = status.bhv_ctx_id;
        msg.trinary.value = bdd_ros2_interfaces::msg::Trinary::TRUE;

        located_pick_pub_->publish(msg);
    }
    if (status.is_obj_held_by_robot) {
        bdd_ros2_interfaces::msg::TrinaryStamped msg;
        msg.stamp = this->now();
        msg.scenario_context_id = status.bhv_ctx_id;
        msg.trinary.value = bdd_ros2_interfaces::msg::Trinary::TRUE;
        
        is_held_pub_->publish(msg);
    }
    if (status.is_obj_located_at_place_location) {
        bdd_ros2_interfaces::msg::TrinaryStamped msg;
        msg.stamp = this->now();
        msg.scenario_context_id = status.bhv_ctx_id;
        msg.trinary.value =bdd_ros2_interfaces::msg::Trinary::TRUE;

        located_place_pub_->publish(msg);
    }
    if (status.is_pick_start) {
        bdd_ros2_interfaces::msg::Event msg;
        msg.stamp = this->now();
        msg.scenario_context_id = status.bhv_ctx_id;
        msg.uri = makeTemplateEventUri("evt-pick-start");
        
        event_pub_->publish(msg);
    }
    if (status.is_pick_end) {
        bdd_ros2_interfaces::msg::Event msg;
        msg.stamp = this->now();
        msg.scenario_context_id = status.bhv_ctx_id;
        msg.uri = makeTemplateEventUri("evt-pick-end");
        
        event_pub_->publish(msg);
    }
    if (status.is_place_start) {
        bdd_ros2_interfaces::msg::Event msg;
        msg.stamp = this->now();
        msg.scenario_context_id = status.bhv_ctx_id;
        msg.uri = makeTemplateEventUri("evt-place-start");
        
        event_pub_->publish(msg);
    }
    if (status.is_place_end) {
        bdd_ros2_interfaces::msg::Event msg;
        msg.stamp = this->now();
        msg.scenario_context_id = status.bhv_ctx_id;
        msg.uri = makeTemplateEventUri("evt-place-end");
        
        event_pub_->publish(msg);
    }
}

int TaskStatusROSNode::send_goal(const control_msgs::action::GripperCommand::Goal& goal_msg)
{
    using namespace std::placeholders;
    printf("Attempting to send gripper command goal with position: %f\n", goal_msg.command.position);

    this->publish_timer_->cancel();

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<GripperCommand>::SendGoalOptions();
    send_goal_options.goal_response_callback = [this](const GoalHandleGripperCommand::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
    };

    // send_goal_options.feedback_callback = [this](
    // GoalHandleGripperCommand::SharedPtr,
    // const std::shared_ptr<const GripperCommand::Feedback> feedback)
    // {
    // std::stringstream ss;
    // ss << "Next number in sequence received: ";
    // for (auto number : feedback->partial_sequence) {
    // ss << number << " ";
    // }
    // RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    // };

    send_goal_options.result_callback = [this](const GoalHandleGripperCommand::WrappedResult & result)
    {
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
        break;
    case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
    case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
    default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    
    auto positions = result.result->position;

    RCLCPP_INFO(this->get_logger(), "Position: %f", positions);
    };
    this->gripper_cmd_goal_publisher->async_send_goal(goal_msg, send_goal_options);
    printf("Gripper command goal sent\n");
    return 0;
}