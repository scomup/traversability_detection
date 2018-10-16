#ifndef FIND_POSE_ON_SURFACE_H_
#define FIND_POSE_ON_SURFACE_H_

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include "Eigen/Core"
#include "Eigen/Geometry"

#include <cstddef>
#include <memory>
#include <type_traits>
#include <utility>

class DrawerForCloudAnalyzer;

template <typename POINT_TYPE>
class CloudAnalyzer
{
  friend class DrawerForCloudAnalyzer;


public:
  CloudAnalyzer(typename pcl::PointCloud<POINT_TYPE>::Ptr pcl_point_cloud);

  bool FindPoseLieOnTheSurface(const Eigen::Matrix4d &in_pose,
                               Eigen::Matrix4d &out_pose);

  bool EstimateTraversability(const POINT_TYPE point);

  bool EstimateTraversability(const Eigen::Vector3d point);

  const typename pcl::PointCloud<POINT_TYPE>::Ptr getcloud(){return pcl_point_cloud_;};


private:

  std::vector<int> FindNearest(const POINT_TYPE point);

  Eigen::Vector4d FindPlane(const POINT_TYPE point,
                            const Eigen::Vector3d &normal_vector);

  Eigen::Vector4d FindPlaneOnCloud(const std::vector<int> &idx,
                            const Eigen::Vector4d &plane);

  double GetDistToPlane(const POINT_TYPE point,
                        const Eigen::Vector4d &plane);

  typename pcl::PointCloud<POINT_TYPE>::Ptr pcl_point_cloud_;
  typename pcl::KdTreeFLANN<POINT_TYPE> kdtree_;
  double max_angle_ = M_PI / 6;
  double max_sd_ = 0.05;
  double radius_ = 0.2;
};

//template class CloudAnalyzer<pcl::PointNormal>;
template class CloudAnalyzer<pcl::PointXYZ>;


#endif // SAMPLE_CARTOCOMMON_MAKE_UNIQUE_H_
