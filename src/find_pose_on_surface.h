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

#include "Eigen/Core"
#include "Eigen/Geometry"

#include <cstddef>
#include <memory>
#include <type_traits>
#include <utility>


class FindPoseOnSurface
{
public:
    FindPoseOnSurface(pcl::PointCloud<pcl::PointNormal>::Ptr pcl_point_cloud, pcl::KdTreeFLANN<pcl::PointNormal> kdtree);
    std::vector<int> FindNearest(pcl::PointNormal searchPoint, float radius);
    double GetNormalVector(pcl::PointNormal searchPoint,
                           std::vector<int> &idx,
                           Eigen::Vector3d &normal_vector);
    void GetCentroidPoint(pcl::PointNormal searchPoint,
                          std::vector<int> &idx,
                          Eigen::Vector3d &centroid_point);

    bool FindPoseLieOnTheSurface(Eigen::Matrix4d &in_pose, Eigen::Matrix4d &out_pose);

  private:

    pcl::PointCloud<pcl::PointNormal>::Ptr pcl_point_cloud_normals_;
    pcl::KdTreeFLANN<pcl::PointNormal> kdtree_;

};


#endif  // SAMPLE_CARTOCOMMON_MAKE_UNIQUE_H_
