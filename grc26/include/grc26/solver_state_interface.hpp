#ifndef SOLVER_STATE_INTERFACE_HPP
#define SOLVER_STATE_INTERFACE_HPP

#include <kdl/jntarray.hpp>
#include "grc26/system_state.hpp"

class SolverStateInterface
{
public:
    explicit SolverStateInterface(unsigned int dof);

    // Access to solver state
    KDL::JntArray& q()        { return q_; }
    KDL::JntArray& qd()       { return qd_; }
    KDL::JntArray& tauCmd()   { return tau_cmd_; }

    const KDL::JntArray& q() const      { return q_; }
    const KDL::JntArray& qd() const     { return qd_; }
    const KDL::JntArray& tauCmd() const { return tau_cmd_; }

    // Mapping
    void toSolver(const SystemState& state);
    void fromSolver(SystemState& state) const;

private:
    KDL::JntArray q_;
    KDL::JntArray qd_;
    KDL::JntArray tau_cmd_;
    unsigned int dof_;
};

#endif // SOLVER_STATE_INTERFACE_HPP