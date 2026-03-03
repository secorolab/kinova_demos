#include "grc26/arm_kinematics.hpp"

ArmKinematics::ArmKinematics(const KDL::Chain& chain)
  : dof_(chain.getNrOfJoints()),
    fk_solver_(chain),
    vel_solver_(chain),
    q_kdl_(dof_),
    qd_kdl_(dof_),
    q_vel_(q_kdl_, qd_kdl_),
    frame_vel_(KDL::Frame::Identity(), KDL::Twist::Zero())
{
}

void ArmKinematics::update(const SystemState& state) noexcept
{
  for (unsigned int i = 0; i < dof_; ++i)
  {
    q_kdl_(i)  = state.arm.q[i];
    qd_kdl_(i) = state.arm.qd[i];
  }

  // Update combined joint array velocity
  q_vel_ = KDL::JntArrayVel(q_kdl_, qd_kdl_);

  fk_solver_.JntToCart(q_kdl_, pose_);
  vel_solver_.JntToCart(q_vel_, frame_vel_);

  twist_ = frame_vel_.GetTwist();
  pose_.M.GetRPY(rpy_[0], rpy_[1], rpy_[2]);
}