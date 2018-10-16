#include <math.h>
#include <stdexcept>

#include "CloudStateSpace.h"

using namespace Eigen;
using namespace std;

#include <iostream>

namespace RRT {

CloudStateSpace::CloudStateSpace(std::shared_ptr<CloudAnalyzer<pcl::PointXYZ>> cloud_analyzer)
    : cloud_analyzer_(cloud_analyzer)
{
    for (auto &point : *cloud_analyzer_->getcloud())
    {
        bounds_.extend(Eigen::Vector3d(point.x, point.y, point.z));
    }
}

bool CloudStateSpace::stateValid(const Vector3d& pt) const {
    if (!VoxelStateSpace::stateValid(pt))
        return false;
    if (cloud_analyzer_->EstimateTraversability(pt))
        return false;
    return true;
}

bool CloudStateSpace::transitionValid(const Vector3d &from,
                                     const Vector3d &to) const
{
    //  make sure we're within bounds
    if (!stateValid(to))
        return false;

    Vector3d delta = to - from;

    double len = delta.norm();

    if (len < min_check_step_)
        return true;

    for (double l = 0; l < len; l += min_check_step_)
    {
        Vector3d mid = from + delta * l;
        if (!stateValid(mid))
            return false;
    }
    return true;
}

}  // namespace RRT

