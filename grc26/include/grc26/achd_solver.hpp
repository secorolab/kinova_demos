#ifndef ACHD_SOLVER_HPP
#define ACHD_SOLVER_HPP


#include <memory>
#include <kdl/chainhdsolver_vereshchagin.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include "kdl/treefksolverpos_recursive.hpp"
#include "kdl/chainhdsolver_vereshchagin_fixed_joint.hpp"
#include "kdl/chainhdsolver_vereshchagin_fext.hpp"
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/frames.hpp>
#include "grc26/system_state.hpp"
#include "grc26/arm_kdl_model.hpp"

#define NUM_JOINTS 7

struct ControlState
{
  KDL::JntArray q;
  KDL::JntArray qd;
  KDL::JntArray tau_mes;

  KDL::JntArray tau_cmd;

  ControlState()
    : q(NUM_JOINTS),
      qd(NUM_JOINTS),
      tau_mes(NUM_JOINTS),
      tau_cmd(NUM_JOINTS)
{}
};


struct State {
    SystemState system_state;

    // Node-local control fields that use KDL types.
    KDL::JntArray qdd{NUM_JOINTS};
    KDL::JntArray tau_cmd{NUM_JOINTS};

    KDL::FrameVel ee_fvel;
    KDL::Twist ee_vel_error;
    KDL::Twist ee_vel_control_signal;

    std::chrono::high_resolution_clock::time_point timestamp;
    uint64_t sequence_number = 0;
};

class VereshchaginSolver
{
public:
    explicit VereshchaginSolver(const ArmKDLModel& model);

    // allocate all buffers and build solver
    bool initialize(unsigned int num_constraints = 6);

    // configuration interface for alpha and beta parameters
    void setAlpha(const KDL::Jacobian& alpha);
    KDL::JntArray& beta() { return beta_; }

    // solve for joint torque commands
    int computeTorques(SystemState& system_state,
                       KDL::JntArray& tau_out);

private:
    const ArmKDLModel& model_;

    std::unique_ptr<KDL::ChainHdSolver_Vereshchagin_Fixed_Joint> solver_;
    std::unique_ptr<KDL::ChainFkSolverVel_recursive> fk_vel_solver_;

    unsigned int dof_{0};
    unsigned int num_segments_{0};
    unsigned int num_constraints_{6};

    // initialization of variables for solver inputs and outputs
    KDL::JntArray q_;
    KDL::JntArray qd_;
    KDL::JntArray qdd_;

    KDL::JntArray tau_cmd_;
    KDL::JntArray ff_taus_;
    KDL::JntArray beta_;

    KDL::Jacobian alpha_;
    KDL::Wrenches f_ext_;

    bool initialized_{false};
};

#endif // ACHD_SOLVER_HPP