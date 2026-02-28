#ifndef CARTESIAN_MOTION_SETPOINT_HPP
#define CARTESIAN_MOTION_SETPOINT_HPP

#include <Eigen/Dense>
#include <vector>

enum class ControlMode {
    None,
    Position,
    Velocity,
    Effort,
    GripperPosition
};

struct CartesianMotionSetpoint {
    ControlMode translation_mode = ControlMode::None;
    ControlMode rotation_mode    = ControlMode::None;
    ControlMode gripper_mode    = ControlMode::None;

    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Vector3d orientation = Eigen::Vector3d::Zero();
    Eigen::Vector3d linear_vel = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_vel = Eigen::Vector3d::Zero();
    Eigen::Vector3d feedforward_force = Eigen::Vector3d::Zero();
    Eigen::Vector3d feedforward_torque = Eigen::Vector3d::Zero();
    double gripper_position = 0.0; // 0.0 (fully open) to 1.0 (fully closed) TODO: confirm the range and direction of gripper position command
};

#endif // CARTESIAN_MOTION_SETPOINT_HPP