#include "grc26/arm_kdl_model.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>

ArmKDLModel::ArmKDLModel() {}

bool ArmKDLModel::loadFromURDF(const std::string& package_name,
                               const std::string& urdf_file,
                               const std::string& chain_root_link,
                               const std::string& chain_tip_link)
{
  try
  {
    std::string urdf_path = ament_index_cpp::get_package_share_directory(package_name)
        + "/urdf/" + urdf_file;
    if (!kdl_parser::treeFromFile(urdf_path, kdl_tree_)){
      std::cerr << "Failed to parse URDF: " << urdf_path << "\n";
      return false;
    }
    if (!kdl_tree_.getChain(chain_root_link, chain_tip_link, kdl_chain_)){
      std::cerr << "Failed to extract KDL chain\n";
      return false;
    }
    std::cout << "KDL model loaded successfully\n";
    std::cout << "DOF: " << num_joints() << " | Segments: " << num_segments() << "\n";

    loaded_ = true;
    return true;
  }
  catch (const std::exception& e)
  {
    std::cerr << "URDF loading exception: " << e.what() << "\n";
  return false;
  }
}