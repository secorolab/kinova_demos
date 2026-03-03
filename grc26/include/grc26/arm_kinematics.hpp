#ifndef ARM_KINEMATICS_HPP
#define ARM_KINEMATICS_HPP

#include <iostream>
#include <array>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
#include "grc26/system_state.hpp"

class SystemState;

class ArmKinematics
{
public:
  ArmKinematics(const KDL::Chain& chain);

  void update(const SystemState& state) noexcept;

  const KDL::Frame& pose() const noexcept { return pose_; }
  const KDL::Twist& twist() const noexcept { return twist_; }
  const double* rpy() const noexcept { return rpy_; }

private:
  unsigned int dof_;

  KDL::ChainFkSolverPos_recursive fk_solver_;
  KDL::ChainFkSolverVel_recursive vel_solver_;

  KDL::JntArray q_kdl_;
  KDL::JntArray qd_kdl_;
  KDL::JntArrayVel q_vel_;
  KDL::FrameVel frame_vel_;

  KDL::Frame pose_;
  KDL::Twist twist_;
  double rpy_[3];
};

#endif