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
                if (std::fabs(pos_error) > task.ee_linear.vel_threshold)
                {
                    double dir = (pos_error > 0.0) ? 1.0 : -1.0;
                    double vref = dir * task.ee_linear.vel_threshold;

                    double error = vref - ee_twist.vel[i];
                    // reset to PID if it is set to zero after switching to stiffness control
                    controllers_.pid_lin[i].set_stiffness_control_mode(false);
                    beta(i) = controllers_.pid_lin[i].control(error, dt);
                }
                else
                {
                    // within threshold, hold position with stiffness controller
                    double error = pos_error;
                    controllers_.pid_lin[i].set_stiffness_control_mode(true);
                    beta(i) = controllers_.pid_lin[i].control(error, dt);
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

            for (int i = 0; i < 3; ++i)
                f_ext[seg](3 + i) = -controllers_.ori_ctrl[i].control(diff(i));

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
}
