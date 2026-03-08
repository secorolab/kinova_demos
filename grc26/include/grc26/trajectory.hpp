#ifndef TRAJECTORY_HPP
#define TRAJECTORY_HPP

#include <memory>
#include "kdl/frames.hpp"
#include "kdl/trajectory_segment.hpp"

class Trajectory {
public:
    Trajectory(
        KDL::Frame start_pose,
        KDL::Frame end_pose,
        double max_vel,
        double max_acc
    );

    KDL::Trajectory_Segment& get() { return *trajectory_; }

private:
    KDL::Frame start_pose_;
    KDL::Frame end_pose_;
    std::unique_ptr<KDL::Trajectory_Segment> trajectory_;

};

#endif // TRAJECTORY_HPP