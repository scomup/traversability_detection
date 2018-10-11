/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

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

template <typename POINT_TYPE>
class CloudEstimator
{
public:
  CloudEstimator(typename pcl::PointCloud<POINT_TYPE>::Ptr pcl_point_cloud,
                    typename pcl::KdTreeFLANN<POINT_TYPE> kdtree);

  std::vector<int> FindNearest(const POINT_TYPE point,
                               const double radius);
  Eigen::Vector4d FindPlane(const POINT_TYPE point,
                            const Eigen::Vector3d &normal_vector);

  Eigen::Vector4d FindPlaneOnCloud(const std::vector<int> &idx,
                            const Eigen::Vector4d &plane);

  double GetDistToPlane(const POINT_TYPE point,
                        const Eigen::Vector4d &plane);

  bool FindPoseLieOnTheSurface(const Eigen::Matrix4d &in_pose,
                               Eigen::Matrix4d &out_pose,
                               const double radius);

  bool EstimateTraversability(const POINT_TYPE point, const double radius);

  const typename pcl::PointCloud<POINT_TYPE>::Ptr getcloud(){return pcl_point_cloud_;};


private:
  typename pcl::PointCloud<POINT_TYPE>::Ptr pcl_point_cloud_;
  typename pcl::KdTreeFLANN<POINT_TYPE> kdtree_;
  double max_angle = M_PI / 6;
  double max_sd = 0.05;
};

template class CloudEstimator<pcl::PointNormal>;

#endif // SAMPLE_CARTOCOMMON_MAKE_UNIQUE_H_
