#include "cloud_estimator.h"
#include <Eigen/Eigenvalues>

template <typename POINT_TYPE>
CloudEstimator<POINT_TYPE>::CloudEstimator(typename pcl::PointCloud<POINT_TYPE>::Ptr pcl_point_cloud,
                                                 typename pcl::KdTreeFLANN<POINT_TYPE> kdtree)
    : pcl_point_cloud_(pcl_point_cloud),
      kdtree_(kdtree) {}

template <typename POINT_TYPE>
std::vector<int> CloudEstimator<POINT_TYPE>::FindNearest(const POINT_TYPE point, const double radius)
{

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    kdtree_.radiusSearch(point, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
    return pointIdxRadiusSearch;
}

template <typename POINT_TYPE>
bool CloudEstimator<POINT_TYPE>::FindPoseLieOnTheSurface(const Eigen::Matrix4d &in_pose,
                                                        Eigen::Matrix4d &out_pose,
                                                        const double radius)
{
    Eigen::Vector3d t = in_pose.block(0, 3, 3, 1);
    POINT_TYPE point;
    point.x = t.x();
    point.y = t.y();
    point.z = t.z();

    std::vector<int> idx = FindNearest(point, radius);
    if (idx.size() < 5)
        return false;
    Eigen::Vector4f plane_parameters_f;
    Eigen::Vector4d plane_parameters;

    float curvature;
    pcl::computePointNormal(*pcl_point_cloud_, idx, plane_parameters_f, curvature);
    plane_parameters = plane_parameters_f.cast<double>();
    auto plane_on_points = FindPlaneOnCloud(idx, plane_parameters);

    auto dist = GetDistToPlane(point, plane_on_points);
    Eigen::Vector3d projected_point = t - dist*plane_parameters.head<3>();

    if (plane_parameters.z() < 0)
    {
        plane_parameters = -plane_parameters;
    }
    out_pose = Eigen::Matrix4d::Identity();
    out_pose.block(0, 3, 3, 1) = projected_point;
    auto z_axis = in_pose.block(0, 2, 3, 1);
    Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(z_axis, plane_parameters.head<3>());
    out_pose.block(0, 0, 3, 3) = q * in_pose.block(0, 0, 3, 3);
    return true;
}

template <typename POINT_TYPE>
Eigen::Vector4d CloudEstimator<POINT_TYPE>::FindPlane(const POINT_TYPE point,
                                                         const Eigen::Vector3d &normal_vector)
{
    double x = point.x;
    double y = point.y;
    double z = point.z;
    double a = normal_vector.x();
    double b = normal_vector.y();
    double c = normal_vector.z();

    return Eigen::Vector4d(a, b, c, -(a * x + b * y + c * z));
}

template <typename POINT_TYPE>
double CloudEstimator<POINT_TYPE>::GetDistToPlane(const POINT_TYPE point,
                                                     const Eigen::Vector4d &plane)
{
    double x = point.x;
    double y = point.y;
    double z = point.z;
    double a = plane[0];
    double b = plane[1];
    double c = plane[2];
    double d = plane[3];

    return (a * x + b * y + c * z + d) / std::sqrt(a * a + b * b + c * c);
}

template <typename POINT_TYPE>
Eigen::Vector4d CloudEstimator<POINT_TYPE>::FindPlaneOnCloud(const std::vector<int> &idx,
                                                                        const Eigen::Vector4d &plane)
{
    double max_dist = -std::numeric_limits<double>::max();
    int max_i = 0;
    for (auto i : idx)
    {
        double dist = GetDistToPlane(pcl_point_cloud_->points[i], plane);
        if (dist > max_dist)
        {
            max_dist = dist;
            max_i = i;
        }
    }
    return FindPlane(pcl_point_cloud_->points[max_i], plane.head(3));
}

template <typename POINT_TYPE>
bool CloudEstimator<POINT_TYPE>::EstimateTraversability(const POINT_TYPE point,
                                                        const double radius)
{
    std::vector<int> idx = FindNearest(point, radius);
    if (idx.size() < 5)
        return false;
    Eigen::Vector4f plane_parameters_f;
    Eigen::Vector4d plane_parameters;

    float curvature;
    pcl::computePointNormal(*pcl_point_cloud_, idx, plane_parameters_f, curvature);
    plane_parameters = plane_parameters_f.cast<double>();
    if (plane_parameters.z() < 0)
    {
        plane_parameters = -plane_parameters;
    }
    Eigen::AngleAxisd angle_axis(Eigen::Quaterniond::FromTwoVectors(
        plane_parameters.head<3>(), Eigen::Vector3d::UnitZ()));
        std::cout<<"angle: "<<angle_axis.angle()<<std::endl;
    if (angle_axis.angle() > max_angle)
        return false;

    std::vector<double> dists;
    double mean = 0;
    
    for (auto i : idx)
    {
        double dist = GetDistToPlane(pcl_point_cloud_->points[i], plane_parameters);
        dists.push_back(dist);
        mean += dist;
    }
    mean /= idx.size();

    double var = 0;
    for (auto dist : dists)
    {
        var += (dist - mean) * (dist - mean);
    }
    var /= dists.size();
    double sd = std::sqrt(var);
    std::cout<<"sd:"<<sd<<std::endl;
    if (sd > max_sd)
        return false;
    return true;
}