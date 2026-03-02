#ifndef ARM_KDL_MODEL_HPP
#define ARM_KDL_MODEL_HPP

#include <string>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl_parser/kdl_parser.hpp>

class ArmKDLModel
{
public:
    ArmKDLModel();

    // Load URDF and build KDL chain
    bool loadFromURDF(const std::string& package_name,
                      const std::string& urdf_file,
                      const std::string& chain_root_link,
                      const std::string& chain_tip_link);

    const KDL::Chain&  chain()   const { return kdl_chain_; }
    const KDL::Tree&   tree()    const { return kdl_tree_;  }
    const KDL::Vector& gravity() const { return gravity_;   }

    unsigned int num_joints() const { return kdl_chain_.getNrOfJoints(); }
    unsigned int num_segments() const { return kdl_chain_.getNrOfSegments(); }

private:
    KDL::Tree  kdl_tree_;
    KDL::Chain kdl_chain_;
    KDL::Vector gravity_{0.0, 0.0, -9.81};

    bool loaded_{false};
};

#endif // ARM_KDL_MODEL_HPP