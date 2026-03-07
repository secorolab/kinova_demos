#include <string>
#include <iostream>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "kdl_parser/kdl_parser.hpp"

#include "kdl/chain.hpp"
#include "kdl/chainhdsolver_vereshchagin.hpp"
#include "kdl/chainhdsolver_vereshchagin_fixed_joint.hpp"
#include "kdl/kinfam_io.hpp"

int main(int argc, char** argv)
{
  (void)argc;
  (void)argv;

  auto pkg_share_dir = ament_index_cpp::get_package_share_directory("grc26");

  std::string urdf_file = pkg_share_dir + "/urdf/GEN3_URDF_V12.urdf";
  // std::string urdf_file = pkg_share_dir + "/urdf/gen3.urdf";

  std::string bracelet_link = "Bracelet_Link";
  std::string bracelet_link2 = "bracelet_link";
  
  std::string ee_link = "EndEffector_Link";
  std::string ee_link2 = "end_effector_link";

  KDL::Tree tree;
  if (!kdl_parser::treeFromFile(urdf_file, tree)) {
    std::cerr << "Failed to construct kdl tree from urdf file" << std::endl;
    return -1;
  }

  KDL::Chain chain, chain_ft;
  
  if (!tree.getChain("base_link", bracelet_link, chain)) {
    std::cerr << "Failed to get kdl chain from tree" << std::endl;
    return -1;
  }

  if (!tree.getChain("base_link", ee_link, chain_ft)) {
    std::cerr << "Failed to get kdl chain from tree" << std::endl;
    return -1;
  }

  KDL::Twist gravity(KDL::Vector(0.0, 0.0, 9.81), KDL::Vector::Zero());

  constexpr int nc = 6;

  assert (chain.getNrOfJoints() == chain_ft.getNrOfJoints());

  int nj = chain.getNrOfJoints();
  int ns = chain.getNrOfSegments();
  int ns_ft = chain_ft.getNrOfSegments();

  KDL::ChainHdSolver_Vereshchagin achd_solver(chain, gravity, nc);
  KDL::ChainHdSolver_Vereshchagin_Fixed_Joint achd_fj_solver(chain, gravity, nc);
  KDL::ChainHdSolver_Vereshchagin_Fixed_Joint achd_fj_ft_solver(chain_ft, gravity, nc);

  KDL::JntArray q(nj), qd(nj), qdd(nj);
  KDL::Wrenches f_ext(ns), f_ext_ft(ns_ft);
  KDL::JntArray ff_tau(nj);
  KDL::JntArray ctau(nj), ctau_ft(nj);

  for (int i = 0; i < ns; ++i) {
    f_ext[i] = KDL::Wrench::Zero();
  }

  for (int i = 0; i < ns_ft; ++i) {
    f_ext_ft[i] = KDL::Wrench::Zero();
  }

  KDL::JntArray beta(nc);

  KDL::Jacobian alpha(nc);
  alpha.setColumn(0, KDL::Twist(KDL::Vector(1.0, 0.0, 0.0), KDL::Vector::Zero()));
  alpha.setColumn(1, KDL::Twist(KDL::Vector(0.0, 1.0, 0.0), KDL::Vector::Zero()));
  alpha.setColumn(2, KDL::Twist(KDL::Vector(0.0, 0.0, 1.0), KDL::Vector::Zero()));
  alpha.setColumn(3, KDL::Twist(KDL::Vector::Zero(), KDL::Vector(1.0, 0.0, 0.0)));
  alpha.setColumn(4, KDL::Twist(KDL::Vector::Zero(), KDL::Vector(0.0, 1.0, 0.0)));
  alpha.setColumn(5, KDL::Twist(KDL::Vector::Zero(), KDL::Vector(0.0, 0.0, 1.0)));

  // [6.281675, 0.263889, 3.141539, 4.013110, 6.283160, 0.959823, 1.570792]
  // q(0) = 0.0;
  q(1) = 0.263889;
  // q(2) = 0.0;
  q(3) = 2.013110;
  // q(4) = 0.0;
  // q(5) = 0.959823;
  // q(6) = -1.570792;

  int res = 0;

  beta(0) = 0.0;
  beta(1) = 0.0;
  beta(2) = 0.0;
  beta(3) = 0.0;
  beta(4) = 0.0;
  beta(5) = 100.0;

  res = achd_solver.CartToJnt(q, qd, qdd, alpha, beta, f_ext, ff_tau, ctau);
  if (res < 0) {
    std::cerr << "Error in achd_solver.CartToJnt: " << res << std::endl;
    return -1;
  }

  std::cout << "achd ctau:       " << ctau << std::endl;

  res = achd_fj_solver.CartToJnt(q, qd, qdd, alpha, beta, f_ext, ff_tau, ctau);
  if (res < 0) {
    std::cerr << "Error in achd_fj_solver.CartToJnt: " << res << std::endl;
    return -1;
  }

  std::cout << "achd fj ctau:    " << ctau << std::endl;

  res = achd_fj_ft_solver.CartToJnt(q, qd, qdd, alpha, beta, f_ext_ft, ff_tau, ctau_ft);
  if (res < 0) {
    std::cerr << "Error in achd_fj_ft_solver.CartToJnt: " << res << std::endl;
    return -1;
  }

  std::cout << "achd fj ft ctau: " << ctau_ft << std::endl;

  // cleanup

  return 0;
}
