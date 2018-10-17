#include "cloud_analyzer.h"
#include <Eigen/Eigenvalues>

template <typename POINT_TYPE>
CloudAnalyzer<POINT_TYPE>::CloudAnalyzer(typename pcl::PointCloud<POINT_TYPE>::Ptr pcl_point_cloud)
    : pcl_point_cloud_(pcl_point_cloud)
{
    kdtree_.setInputCloud(pcl_point_cloud_);
}

template <typename POINT_TYPE>
std::vector<int> CloudAnalyzer<POINT_TYPE>::FindNearest(const POINT_TYPE point)
{

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    kdtree_.radiusSearch(point, radius_, pointIdxRadiusSearch, pointRadiusSquaredDistance);
    return pointIdxRadiusSearch;
}


template <typename POINT_TYPE>
bool CloudAnalyzer<POINT_TYPE>::FindPointLieOnTheSurface(const Eigen::Vector3d &in_point,
                                                         Eigen::Vector3d &out_point)
{
    POINT_TYPE point;
    point.x = in_point.x();
    point.y = in_point.y();
    point.z = in_point.z();

    std::vector<int> idx = FindNearest(point);
    if (idx.size() < 5){
        return false;
    }
    Eigen::Vector4f plane_parameters;

    float curvature;
    pcl::computePointNormal(*pcl_point_cloud_, idx, plane_parameters, curvature);
    auto plane_on_points = FindPlaneOnCloud(idx, plane_parameters.cast<double>());

    auto dist = GetDistToPlane(point, plane_on_points);
    out_point = in_point - dist*plane_parameters.head<3>().cast<double>();

    return true;
}

template <typename POINT_TYPE>
bool CloudAnalyzer<POINT_TYPE>::FindPoseLieOnTheSurface(const Eigen::Matrix4d &in_pose,
                                                        Eigen::Matrix4d &out_pose)
{
    Eigen::Vector3d t = in_pose.block(0, 3, 3, 1);
    POINT_TYPE point;
    point.x = t.x();
    point.y = t.y();
    point.z = t.z();

    std::vector<int> idx = FindNearest(point);
    if (idx.size() < 5)
        return false;
    Eigen::Vector4f plane_parameters;

    float curvature;
    pcl::computePointNormal(*pcl_point_cloud_, idx, plane_parameters, curvature);
    auto plane_on_points = FindPlaneOnCloud(idx, plane_parameters.cast<double>());

    auto dist = GetDistToPlane(point, plane_on_points);
    Eigen::Vector3d projected_point = t - dist*plane_parameters.head<3>().cast<double>();

    if (plane_parameters.z() < 0)
    {
        plane_parameters = -plane_parameters;
    }
    out_pose = Eigen::Matrix4d::Identity();
    out_pose.block(0, 3, 3, 1) = projected_point;
    auto z_axis = in_pose.block(0, 2, 3, 1);
    Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(z_axis, plane_parameters.head<3>().cast<double>());
    out_pose.block(0, 0, 3, 3) = q * in_pose.block(0, 0, 3, 3);
    return true;
}

template <typename POINT_TYPE>
Eigen::Vector4d CloudAnalyzer<POINT_TYPE>::FindPlane(const POINT_TYPE point,
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
double CloudAnalyzer<POINT_TYPE>::GetDistToPlane(const POINT_TYPE point,
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
Eigen::Vector4d CloudAnalyzer<POINT_TYPE>::FindPlaneOnCloud(const std::vector<int> &idx,
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
bool CloudAnalyzer<POINT_TYPE>::EstimateTraversability(const Eigen::Vector3d point)
{
    POINT_TYPE search_point;
    search_point.x = point.x();
    search_point.y = point.y();
    search_point.z = point.z();
    return EstimateTraversability(search_point);
}

template <typename POINT_TYPE>
bool CloudAnalyzer<POINT_TYPE>::EstimateTraversability(const POINT_TYPE point)
{
    std::vector<int> idx = FindNearest(point);
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
        //std::cout<<"angle: "<<angle_axis.angle()<<std::endl;
    if (angle_axis.angle() > max_angle_)
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
    //std::cout<<"sd:"<<sd<<std::endl;
    if (sd > max_sd_)
        return false;
    return true;
}