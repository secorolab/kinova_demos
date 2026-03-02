#include "grc26/achd_solver.hpp"
#include <iostream>

VereshchaginSolver::VereshchaginSolver(const ArmKDLModel& model)
    : model_(model)
{}

void VereshchaginSolver::setAlpha(const KDL::Jacobian& alpha)
{
  if (alpha.columns() != num_constraints_)
    throw std::runtime_error("Alpha dimension mismatch");
  alpha_ = alpha;
}

bool VereshchaginSolver::initialize(unsigned int num_constraints)
{
  num_constraints_ = num_constraints;
  dof_          = model_.num_joints();
  num_segments_ = model_.num_segments();

  if (dof_ == 0)
  {
    std::cerr << "Solver init failed: model not loaded\n";
    return false;
  }

  // Preallocate joint arrays
  q_       = KDL::JntArray(dof_);
  qd_      = KDL::JntArray(dof_);
  qdd_     = KDL::JntArray(dof_);
  tau_cmd_ = KDL::JntArray(dof_);
  ff_taus_ = KDL::JntArray(dof_);
  beta_    = KDL::JntArray(num_constraints_);

  alpha_ = KDL::Jacobian(num_constraints_);
  f_ext_ = KDL::Wrenches(num_segments_);
  KDL::Vector gravity_vec = model_.gravity();

  // Gravity twist. As solver expects gravity to be in the negative direction, the gravity vector is negated here.
  KDL::Twist g_twist(
    KDL::Vector(-gravity_vec.x(), -gravity_vec.y(), -gravity_vec.z()),
    KDL::Vector::Zero()
  );

  solver_ = std::make_unique<KDL::ChainHdSolver_Vereshchagin_Fixed_Joint>(
          model_.chain(), g_twist, num_constraints_);

  fk_vel_solver_ = std::make_unique<KDL::ChainFkSolverVel_recursive>(
          model_.chain());

  // set zero external forces
  for (size_t i = 0; i < f_ext_.size(); ++i) {
    f_ext_[i] = KDL::Wrench::Zero();
  }

  initialized_ = true;
  return true;
}

int VereshchaginSolver::computeTorques(SystemState& system_state,
                                       KDL::JntArray& tau_out)
{
  if (!initialized_)
    return -1;

  // Copy state into preallocated arrays
  for (unsigned int i = 0; i < dof_; ++i)
  {
    q_(i)   = system_state.arm.q[i];
    qd_(i)  = system_state.arm.qd[i];
    qdd_(i) = system_state.arm.qdd[i];
  }

  int r = solver_->CartToJnt(q_,
                            qd_,
                            qdd_,
                            alpha_,
                            beta_,
                            f_ext_,
                            ff_taus_,
                            tau_cmd_);
  for (unsigned int i = 0; i < 7; ++i) {
    system_state.arm.tau_cmd[i] = tau_cmd_(i);
  }

  return r;
}