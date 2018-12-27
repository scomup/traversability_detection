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




class CloudAnalyzerHandle;

template <typename POINT_TYPE>
class CloudAnalyzer
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  friend class CloudAnalyzerHandle;

  enum TRAVERSABILITY
  {
    TRAVERSABLE = 100,
    ADJACENTED = 50,
    UNTRAVERSABLE = 0,
    IGNORE = 200
  };

public:

  CloudAnalyzer(typename pcl::PointCloud<POINT_TYPE>::Ptr pcl_point_cloud);

  bool FindPoseLieOnTheSurface(const Eigen::Matrix4d &in_pose,
                               Eigen::Matrix4d &out_pose) const;

  bool FindPointLieOnTheSurface(const Eigen::Vector3d &in_point,
                                Eigen::Vector3d &out_point) const;

  bool EstimateTraversability(const POINT_TYPE point) const;

  bool EstimateTraversability(const Eigen::Vector3d point) const;

  bool EstimateTraversabilityLite(const POINT_TYPE point) const;

  bool EstimateTraversabilityLite(const Eigen::Vector3d point) const;

  std::vector<int> FindPointsInRadius(const Eigen::Vector3d point, const float radius) const;

  const typename pcl::PointCloud<POINT_TYPE>::Ptr getcloud(){return pcl_point_cloud_;};


private:

  int FindNearest(const POINT_TYPE point) const;

  std::vector<int> FindPointsInRadius(const POINT_TYPE point, float radius) const;

  Eigen::Vector4d FindPlane(const POINT_TYPE point,
                            const Eigen::Vector3d &normal_vector) const ;

  Eigen::Vector4d FindPlaneOnCloud(const std::vector<int> &idx,
                                   const Eigen::Vector4d &plane) const;

  double GetDistToPlane(const POINT_TYPE point,
                        const Eigen::Vector4d &plane) const;

  void EstimateNormals();

  void EstimateTraversability();



  typename pcl::PointCloud<POINT_TYPE>::Ptr pcl_point_cloud_;
  typename pcl::KdTreeFLANN<POINT_TYPE> kdtree_;
  pcl::PointCloud<pcl::Normal>::Ptr normal_;
  std::vector<uint8_t> traversability_;
  std::vector<double> angles_;
  std::vector<std::vector<int>> idxs_;
  const double max_angle_ = M_PI/6;
  const double max_sd_ = M_PI/24;
  const double normal_radius_ = 0.3;
  const double radius_ = 0.2;
};

//template class CloudAnalyzer<pcl::PointNormal>;
template class CloudAnalyzer<pcl::PointXYZ>;


#endif // SAMPLE_CARTOCOMMON_MAKE_UNIQUE_H_
