#include "grc26/solver_state_interface.hpp"
#include <stdexcept>

SolverStateInterface::SolverStateInterface(unsigned int dof)
    : q_(dof),
      qd_(dof),
      tau_cmd_(dof),
      dof_(dof)
{
}

void SolverStateInterface::toSolver(const SystemState& state)
{
  for (unsigned int i = 0; i < dof_; ++i)
  {
    q_(i)  = state.arm.q[i];
    qd_(i) = state.arm.qd[i];
  }
}

void SolverStateInterface::fromSolver(SystemState& state) const
{
  for (unsigned int i = 0; i < dof_; ++i)
  {
    state.arm.tau_cmd[i] = tau_cmd_(i);
  }
}