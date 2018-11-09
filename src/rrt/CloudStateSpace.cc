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
    if (cloud_analyzer_->EstimateTraversabilityLite(pt))
        return true;
    return false;
}

Eigen::Vector3d CloudStateSpace::intermediateState(const Eigen::Vector3d &source,
                                                   const Eigen::Vector3d &target,
                                                   double stepSize) const
{
    Eigen::Vector3d delta = target - source;
    if(delta.norm() == 0)
        return source;
    delta = delta / delta.norm(); //  unit vector
    Eigen::Vector3d val = source + delta * stepSize;
    Eigen::Vector3d new_point;
    cloud_analyzer_->FindPointLieOnTheSurface(val, new_point);
    return new_point;
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
        Vector3d mid = from + delta * l/ len;
        Eigen::Vector3d mid_on_suf;
        cloud_analyzer_->FindPointLieOnTheSurface(mid, mid_on_suf);

        if (!stateValid(mid_on_suf))
            return false;
    }
    return true;
}

Eigen::Vector3d CloudStateSpace::randomState() const
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = cloud_analyzer_->getcloud();
    int i = rand() % cloud->size();
    Eigen::Vector3d state(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
    return state;
}

std::vector<Eigen::Vector3d> CloudStateSpace::radiusRandomState(const Eigen::Vector3d point, double radius) const
{
    const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = cloud_analyzer_->getcloud();
    std::vector<int> idx = cloud_analyzer_->FindPointsInRadius(point, radius);
    std::vector<Eigen::Vector3d> states;
    for (auto i : idx)
    {
        if(cloud_analyzer_->EstimateTraversabilityLite(cloud->points[i]))
            states.emplace_back(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
    }
    return states;
}


}  // namespace RRT


