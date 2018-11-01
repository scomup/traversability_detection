#include "cloud_analyzer.h"
#include <pcl/features/normal_3d.h>
#include <Eigen/Eigenvalues>
#include <chrono>

template <typename POINT_TYPE>
CloudAnalyzer<POINT_TYPE>::CloudAnalyzer(typename pcl::PointCloud<POINT_TYPE>::Ptr pcl_point_cloud)
    : pcl_point_cloud_(pcl_point_cloud)
{
    kdtree_.setInputCloud(pcl_point_cloud_);
    //auto s1 = std::chrono::system_clock::now();

    EstimateNormals();
    //auto s2 = std::chrono::system_clock::now();
    //std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(s2 - s1).count() << " milli sec \n";

    EstimateTraversability();
    //auto s3 = std::chrono::system_clock::now();
    //std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(s3 - s2).count() << " milli sec \n";
}

template <typename POINT_TYPE>
void CloudAnalyzer<POINT_TYPE>::EstimateNormals()
{
    pcl::NormalEstimation<POINT_TYPE, pcl::Normal> ne;
    ne.setInputCloud(pcl_point_cloud_);
    typename pcl::search::KdTree<POINT_TYPE>::Ptr tree(new pcl::search::KdTree<POINT_TYPE>());
    ne.setSearchMethod(tree);
    normal_ = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch(radius_);
    ne.compute(*normal_);
    for (auto &n : *normal_)
    {
        if (n.normal[2] < 0)
        {
            n.normal[0] = -n.normal[0];
            n.normal[1] = -n.normal[1];
            n.normal[2] = -n.normal[2];
        }
        Eigen::Vector3d norm(n.normal[0],
                             n.normal[1],
                             n.normal[2]);
        Eigen::AngleAxisd angle_axis(Eigen::Quaterniond::FromTwoVectors(
            norm, Eigen::Vector3d::UnitZ()));
        angles_.push_back(angle_axis.angle());
    }
}

template <typename POINT_TYPE>
void CloudAnalyzer<POINT_TYPE>::EstimateTraversability()
{
    for (size_t i = 0; i < pcl_point_cloud_->size(); i++)
    {
        if (angles_[i] > max_angle_)
        {
            traversability_.push_back(UNTRAVERSABLE);
            continue;
        }

        std::vector<int> idx = FindPointsInRadius(pcl_point_cloud_->points[i], radius_);
        if (idx.size() < 5)
        {
            traversability_.push_back(UNTRAVERSABLE);
            continue;
        }

        double mean = 0;
        for (auto i : idx)
        {
            mean += angles_[i];
        }
        mean /= idx.size();

        double var = 0;
        for (auto i : idx)
        {
            var += (angles_[i] - mean) * (angles_[i] - mean);
        }
        var /= idx.size();

        double sd = std::sqrt(var);
        if (sd > max_sd_)
        {
            traversability_.push_back(UNTRAVERSABLE);
        }
        else
        {
            traversability_.push_back(TRAVERSABLE);
        }
    }

    for (size_t i = 0; i < pcl_point_cloud_->size(); i++)
    {
        if (traversability_[i] != UNTRAVERSABLE)
            continue;
        std::vector<int> idx = FindPointsInRadius(pcl_point_cloud_->points[i], radius_);
        for (auto j : idx)
        {
            if (traversability_[j] == TRAVERSABLE)
            {
                traversability_[i] = ADJACENTED;
                continue;
            }
        }
    }
}

template <typename POINT_TYPE>
std::vector<int> CloudAnalyzer<POINT_TYPE>::FindPointsInRadius(const POINT_TYPE point, float radius) const
{
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    kdtree_.radiusSearch(point, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
    return pointIdxRadiusSearch;
}

template <typename POINT_TYPE>
int CloudAnalyzer<POINT_TYPE>::FindNearest(const POINT_TYPE point) const
{
    int K = 1;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    if (kdtree_.nearestKSearch(point, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
    {
        return pointIdxNKNSearch[0];
    }
    return -1;
}


template <typename POINT_TYPE>
bool CloudAnalyzer<POINT_TYPE>::FindPointLieOnTheSurface(const Eigen::Vector3d &in_point,
                                                         Eigen::Vector3d &out_point) const
{
    POINT_TYPE point;
    point.x = in_point.x();
    point.y = in_point.y();
    point.z = in_point.z();

    std::vector<int> idx = FindPointsInRadius(point, radius_);
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
                                                        Eigen::Matrix4d &out_pose) const
{
    Eigen::Vector3d t = in_pose.block(0, 3, 3, 1);
    POINT_TYPE point;
    point.x = t.x();
    point.y = t.y();
    point.z = t.z();

    std::vector<int> idx = FindPointsInRadius(point, radius_);
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
                                                         const Eigen::Vector3d &normal_vector) const
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
                                                     const Eigen::Vector4d &plane) const
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
                                                                        const Eigen::Vector4d &plane) const
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
bool CloudAnalyzer<POINT_TYPE>::EstimateTraversability(const Eigen::Vector3d point) const
{
    //compute the traversability for arbitrary position
    POINT_TYPE search_point;
    search_point.x = point.x();
    search_point.y = point.y();
    search_point.z = point.z();
    return EstimateTraversability(search_point);
}

template <typename POINT_TYPE>
bool CloudAnalyzer<POINT_TYPE>::EstimateTraversability(const POINT_TYPE point) const
{
    std::vector<int> idx = FindPointsInRadius(point, radius_);
    if (idx.size() < 5)
    {
        return false;
    }

    Eigen::Vector4f plane_parameters;

    float curvature;
    pcl::computePointNormal(*pcl_point_cloud_, idx, plane_parameters, curvature);
    if (plane_parameters.z() < 0)
    {
        plane_parameters = -plane_parameters;
    }

    Eigen::AngleAxisd angle_axis(Eigen::Quaterniond::FromTwoVectors(
        plane_parameters.head<3>().cast<double>(), Eigen::Vector3d::UnitZ()));
    if (angle_axis.angle() > max_angle_)
        return false;

    std::vector<double> angles;
    double mean = 0;
    for (auto i : idx)
    {
        Eigen::Vector3d n(normal_->points[i].normal[0],normal_->points[i].normal[1],normal_->points[i].normal[2]);
        Eigen::AngleAxisd angle_axis(Eigen::Quaterniond::FromTwoVectors(
            plane_parameters.head<3>().cast<double>(), n));
        angles.push_back(angle_axis.angle());
        mean += angle_axis.angle();
    }
    mean /= idx.size();

    double var = 0;
    for (auto a : angles)
    {
        var += (a - mean) * (a - mean);
    }
    var /= angles.size();
    double sd = std::sqrt(var);
    if (sd > max_sd_)
        return false;
    return true;
}

template <typename POINT_TYPE>
bool CloudAnalyzer<POINT_TYPE>::EstimateTraversabilityLite(const Eigen::Vector3d point) const
{
    POINT_TYPE search_point;
    search_point.x = point.x();
    search_point.y = point.y();
    search_point.z = point.z();
    return EstimateTraversabilityLite(search_point);
}

template <typename POINT_TYPE>
bool CloudAnalyzer<POINT_TYPE>::EstimateTraversabilityLite(const POINT_TYPE point) const
{
    int id = FindNearest(point);
    if (id == -1)
        return false;
    if(traversability_[id] != TRAVERSABLE){
        return false;
    }
    return true;
}