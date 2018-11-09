#pragma once

#include <memory>
#include <Eigen/Dense>
//#include <src/rrt/ObstacleGrid.hpp>
#include "VoxelStateSpace.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "../cloud_analyzer.h"


namespace RRT {

/**
 * @brief A 2d plane with continuous states and discretized obstacles.
 * @details The state space is broken up into a grid with the given discrete
 * height and widths.
 */
class CloudStateSpace : public VoxelStateSpace<Eigen::Vector3d> {
public:
  CloudStateSpace(std::shared_ptr<CloudAnalyzer<pcl::PointXYZ>> cloud_analyzer);

  /**
     * Returns a boolean indicating whether the given point is within bounds and
     * obstacle-free.
     */
  bool stateValid(const Eigen::Vector3d &pt) const;

  bool transitionValid(const Eigen::Vector3d& from,
                       const Eigen::Vector3d& to) const;

  //Eigen::Vector3d intermediateState(const Eigen::Vector3d& source,
  //                                  const Eigen::Vector3d& target,
  //                                  double minStepSize,
  //                                  double maxStepSize) const;

  //const ObstacleGrid& obstacleGrid() const;
  //ObstacleGrid& obstacleGrid();


  Eigen::Vector3d intermediateState(const Eigen::Vector3d &source,
                                    const Eigen::Vector3d &target,
                                    double stepSize) const;

  Eigen::Vector3d randomState() const;

  std::vector<Eigen::Vector3d> radiusRandomState(const Eigen::Vector3d point, double radius) const;


private:
  std::shared_ptr<CloudAnalyzer<pcl::PointXYZ>> cloud_analyzer_;
  double min_check_step_ = 0.1; //todo: move to config file
};

}  // namespace RRT
