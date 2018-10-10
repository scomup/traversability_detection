#include "find_pose_on_surface.h"
#include <Eigen/Eigenvalues> 

FindPoseOnSurface::FindPoseOnSurface(pcl::PointCloud<pcl::PointNormal>::Ptr pcl_point_cloud,
                                     pcl::KdTreeFLANN<pcl::PointNormal> kdtree) : pcl_point_cloud_normals_(pcl_point_cloud),
                                                                                  kdtree_(kdtree) {}


std::vector<int> FindPoseOnSurface::FindNearest(pcl::PointNormal searchPoint, float radius)
{

  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;
  kdtree_.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
  return pointIdxRadiusSearch;
}

double FindPoseOnSurface::GetNormalVector(pcl::PointNormal searchPoint,
                                        std::vector<int> &idx,
                                        Eigen::Vector3d &normal_vector)
{
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero(3, 3);

    for (size_t i = 0; i < idx.size(); ++i)
    {
        double x = pcl_point_cloud_normals_->points[idx[i]].x;
        double y = pcl_point_cloud_normals_->points[idx[i]].y;
        double z = pcl_point_cloud_normals_->points[idx[i]].z;
        Eigen::Vector3d p_diff(searchPoint.x - x, searchPoint.y - y, searchPoint.z - z);
        cov += p_diff * p_diff.transpose();
    }
    cov /= idx.size();
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(cov);
    if (eigensolver.info() != Eigen::Success)
        abort();
    normal_vector = eigensolver.eigenvectors().block(0, 0, 3, 1);
    return eigensolver.eigenvalues()(0);
}


void FindPoseOnSurface::GetCentroidPoint(pcl::PointNormal searchPoint,
                                        std::vector<int> &idx,
                                        Eigen::Vector3d &centroid_point)
{
    centroid_point = Eigen::Vector3d::Zero(3);
    for (size_t i = 0; i < idx.size(); ++i)
    {
        double x = pcl_point_cloud_normals_->points[idx[i]].x;
        double y = pcl_point_cloud_normals_->points[idx[i]].y;
        double z = pcl_point_cloud_normals_->points[idx[i]].z;
        centroid_point += Eigen::Vector3d(x, y, z);;
    }
    centroid_point /= idx.size();
}

bool FindPoseOnSurface::FindPoseLieOnTheSurface(Eigen::Matrix4d &in_pose,
                                                Eigen::Matrix4d &out_pose)
{
    Eigen::Vector3d t = in_pose.block(0,3,3,1);
    pcl::PointNormal searchPoint;
    searchPoint.x = t.x();
    searchPoint.y = t.y();
    searchPoint.z = t.z();

    std::vector<int> idx = FindNearest(searchPoint, 0.3);
    if (idx.size() < 5)
        return false;
    Eigen::Vector3d normal_vector;
    Eigen::Vector3d centroid_point;
    std::cout<<GetNormalVector(searchPoint, idx, normal_vector)<<std::endl;
    GetCentroidPoint(searchPoint, idx, centroid_point);

    if(normal_vector.z() < 0){
        normal_vector = -normal_vector;
    }
    out_pose = Eigen::Matrix4d::Identity();
    out_pose.block(0,3,3,1) = t;
    out_pose(2,3) = centroid_point.z();
    //double dist = (centroid_point - t).norm();
    //out_pose = Eigen::Matrix4d::Identity();
    //out_pose.block(0,3,3,1) = t + dist * normal_vector;

    auto z_axis = in_pose.block(0,2,3,1);
    Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(z_axis, normal_vector);
    out_pose.block(0,0,3,3) = q * in_pose.block(0,0,3,3);
    return true;

/*
    (centroid_point - t)

    Eigen::AngleAxisd angle_axis(Eigen::Quaterniond::FromTwoVectors(
        normal_vector, Eigen::Vector3d::UnitZ()));
    angle_axis.anlge() > 0.3
    */
}
