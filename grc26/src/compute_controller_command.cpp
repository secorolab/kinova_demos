#include "grc26/compute_controller_command.hpp"


ComputeControllerCommand::ComputeControllerCommand(const Controllers& controllers)
: controllers_(controllers)
{
}

void ComputeControllerCommand::compute(
    const SystemState& state,
    const ArmKinematics& kin,
    const TaskSpec& task,
    KDL::JntArray& beta,
    KDL::Wrenches& f_ext,
    double dt)
{
    if (dt <= 0.0)
        dt = (state.arm.cycle_time > 0.0) ? state.arm.cycle_time : 1e-3;

    // ---- reset outputs ----
    for (unsigned int i = 0; i < beta.rows(); ++i)
        beta(i) = 0.0;

    for (size_t i = 0; i < f_ext.size(); ++i)
        f_ext[i] = KDL::Wrench::Zero();

    const KDL::Vector& ee_pos   = kin.pose().p;
    const KDL::Twist&  ee_twist = kin.twist();

    // ---- Linear Control ----
    if (task.ee_linear.enabled)
    {
        for (int i = 0; i < 3; ++i)
        {
            switch (task.ee_linear.mode[i])
            {
            case LinearMode::Velocity:
            {
                double err = task.ee_linear.velocity[i] - ee_twist.vel[i];
                beta(i) = controllers_.pid_lin[i].control(err, dt);
                break;
            }

            case LinearMode::Position:
            {
                double pos_error = task.ee_linear.position[i] - ee_pos[i];
                // clamp pos_error with vref
                pos_error = std::max(std::min(pos_error, task.ee_linear.vel_threshold), -task.ee_linear.vel_threshold);

                beta(i) = controllers_.pid_lin[i].control(pos_error, dt);
                if (i == 2)
                {
                    printf("Position error in z: %f, velocity: %f, beta: %f\n", pos_error, ee_twist.vel[i], beta(i));
                }
                else
                {
                    printf("Position error in axis %d: %f, velocity: %f, beta: %f\n", i, pos_error, ee_twist.vel[i], beta(i));
                }
                break;
            }

            case LinearMode::Force:
            {
                f_ext.back()(i) = task.ee_linear.force[i];
                break;
            }

            case LinearMode::None:
            {
                break;
            }

            default:
                break;
            }
        }
    }

    // ---- Orientation Control ----
    if (task.orientation.enabled)
    {
        switch (task.orientation.mode)
        {
        case OrientationMode::Position:
        {
            KDL::Vector diff = KDL::diff(
                                        kin.pose().M,
                                        KDL::Rotation::RPY(
                                        task.orientation.rpy[0],
                                        task.orientation.rpy[1],
                                        task.orientation.rpy[2]));

            int seg = task.orientation.segment_index;

            for (int i = 0; i < 3; ++i){
                if (seg == 7) // if controlling at the end-effector, using beta
                    beta(3 + i) = controllers_.ori_ctrl[i].control(diff(i));
                else // if controlling at a different segment, usoing fext interface to apply torques
                {
                    f_ext[seg](3 + i) = controllers_.ori_ctrl[i].control(diff(i));
                }
            }

            break;
        }

        case OrientationMode::Velocity:
        {
            int seg = task.orientation.segment_index;

            for (int i = 0; i < 3; ++i){
                if (seg == 7) // if controlling at the end-effector, using beta
                    beta(3 + i) = controllers_.ori_ctrl[i].control(task.orientation.ang_vel[i] - ee_twist.rot[i]);
                else // if controlling at a different segment, we use the velocity error to compute desired angular velocity for damping control
                    f_ext[seg](3 + i) = -controllers_.ori_ctrl[i].control(task.orientation.ang_vel[i] - ee_twist.rot[i]);
            }
            break;
        }

        case OrientationMode::Torque:
        {
            int seg = task.orientation.segment_index;

            for (int i = 0; i < 3; ++i)
                f_ext[seg](3 + i) = -task.orientation.torque[i];

            break;
        }

        case OrientationMode::None:
        {
            break;
        }

        default:
            break;
        }
    }

    // -------- Additional Link Forces --------
    if (task.link_force.enabled)
    {
        int seg = task.link_force.segment_index;
        for (int i = 0; i < 3; ++i)
        {
            f_ext[seg](i) = - task.link_force.force[i];
        }
    }

    // By default, control the forearm to have same yaw throughout the task
    if (task.forearm_yaw_control_enabled)
    {
        double forearm_link_y_axis_angle_sp = 0.0;
        double stiffness_forearm_y_axis_angle = 50.0;
        double deadband_forearm_y_axis_angle = 0.0;
        double torque_limit_forearm_link = 25.0;

        KDL::Vector torque_vector = KDL::Vector::Zero();

        // get y-axis of link3 (forearm) in arm base_link frame
        KDL::Vector measured_ForeArm_Link_y_axis_BL = kin.forearmPoseBL().M.UnitY();

        if (false) printf("Forearm pose: position [%f, %f, %f]\n", kin.forearmPoseBL().p.x(), kin.forearmPoseBL().p.y(), kin.forearmPoseBL().p.z());

        // project y-axis onto xy-plane of arm base_link to get angle made with the plane
        KDL::Vector measured_ForeArm_Link_y_axis_BL_projection_to_XY_plane = KDL::Vector(measured_ForeArm_Link_y_axis_BL.x(), measured_ForeArm_Link_y_axis_BL.y(), 0.0);
        measured_ForeArm_Link_y_axis_BL_projection_to_XY_plane = measured_ForeArm_Link_y_axis_BL_projection_to_XY_plane / measured_ForeArm_Link_y_axis_BL_projection_to_XY_plane.Norm();
        double angle = std::acos(KDL::dot(measured_ForeArm_Link_y_axis_BL, measured_ForeArm_Link_y_axis_BL_projection_to_XY_plane));

        // get torque axis as the cross product between y-axis and its projection vector. This will change direction when the forearm y-axis goes above or below the xy-plane
        KDL::Vector torque_axis = measured_ForeArm_Link_y_axis_BL * measured_ForeArm_Link_y_axis_BL_projection_to_XY_plane;
        torque_axis = torque_axis / torque_axis.Norm();

        double error = 0;
        if (angle < (forearm_link_y_axis_angle_sp - deadband_forearm_y_axis_angle)) {
        error = (forearm_link_y_axis_angle_sp - deadband_forearm_y_axis_angle) - angle;
        } else if (angle > (forearm_link_y_axis_angle_sp + deadband_forearm_y_axis_angle)) {
        error = (forearm_link_y_axis_angle_sp + deadband_forearm_y_axis_angle) - angle;
        } else {
        error = 0;
        }
        double torque_magnitude = std::abs(stiffness_forearm_y_axis_angle * error);
        if (torque_magnitude > torque_limit_forearm_link) {
        torque_magnitude = torque_limit_forearm_link;
        } else if (torque_magnitude < -torque_limit_forearm_link) {
        torque_magnitude = -torque_limit_forearm_link;
        }

        // transform torque axis to forearm link frame
        KDL::Vector torque_axis_forearm_link = kin.forearmPoseBL().M.Inverse() * torque_axis;

        // achd solver takes wrenches in base link frame
        torque_vector = torque_magnitude * torque_axis;

        if (false) printf("Forearm yaw control: angle %f, error %f, torque magnitude %f, torque axis in BL [%f, %f, %f], torque vector in BL [%f, %f, %f]\n",
        angle, error, torque_magnitude,
        torque_axis.x(), torque_axis.y(), torque_axis.z(),
        torque_vector.x(), torque_vector.y(), torque_vector.z());

        int forearm_seg = 3; // forearm link segment index. Also hard-coded in arm_kinematics.cpp when computing forearm pose
        f_ext[forearm_seg-1](3) = - torque_vector.x();
        f_ext[forearm_seg-1](4) = - torque_vector.y();
        f_ext[forearm_seg-1](5) = - torque_vector.z();
    }
}
