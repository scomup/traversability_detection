#pragma once

#include <Eigen/Dense>
#include "StateSpace.h"

namespace RRT {

/**
 * @brief A 3d voxel with continuous states and no obstacles.
 */
template <class POINT_CLASS = Eigen::Vector3d>
class VoxelStateSpace : public StateSpace<POINT_CLASS> {
public:

    VoxelStateSpace(){}

    VoxelStateSpace(Eigen::AlignedBox<double,3> bounds)
        : bounds_(bounds) {}

    POINT_CLASS randomState() const {
        return bounds_.sample();
    }

    POINT_CLASS intermediateState(const POINT_CLASS& source,
                                  const POINT_CLASS& target,
                                  double stepSize) const {
        POINT_CLASS delta = target - source;
        delta = delta / delta.norm();  //  unit vector

        POINT_CLASS val = source + delta * stepSize;
        return val;
    }

    double distance(const POINT_CLASS& from, const POINT_CLASS& to) const {
        POINT_CLASS delta = from - to;
        return delta.norm();
    }

    /**
     * Returns a boolean indicating whether the given point is within bounds.
     */
    bool stateValid(const POINT_CLASS& pt) const {
        return bounds_.contains(pt);
    }


protected:
    Eigen::AlignedBox<double,3> bounds_;
};

}  // namespace RRT
