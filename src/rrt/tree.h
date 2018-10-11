#ifndef RRT_TREE_H_
#define RRT_TREE_H_

#include "node.h"
#include "Eigen/Core"

namespace RRT
{

template <typename POSTION_TYPE, typename POINT_TYPE>
class Tree {
public:
  Tree(const Tree &) = delete;
  Tree &operator=(const Tree &) = delete;
  Tree(std::shared_ptr<CloudEstimator<POINT_TYPE>> cloud_estimator)
      : cloud_estimator_(cloud_estimator),
        kdtree_(flann::KDTreeSingleIndexParams())
  {
      auto cloud = cloud_estimator_->getcloud(); 
      for(const auto &point : *cloud){
          Eigen::Vector3d p(point.x, point.y, point.z);
          box_.extend(p);
      }
  }

  Node<Tree> *grow()
  {
      //return extend(_stateSpace->randomState());
  }

  Node<POSTION_TYPE> *nearest(const POSTION_TYPE &positon, double *distanceOut = nullptr)
  {
      Node<POSTION_TYPE> *best = nullptr;

      // k-NN search (O(log(N)))
      flann::Matrix<double> query;

      query = flann::Matrix<double>((double *)&positon, 1,
                                    sizeof(positon) / sizeof(0.0));
      std::vector<int> i(query.rows);
      flann::Matrix<int> indices(i.data(), query.rows, 1);
      std::vector<double> d(query.rows);
      flann::Matrix<double> dists(d.data(), query.rows, 1);
      int n =
          kdtree_.knnSearch(query, indices, dists, 1, flann::SearchParams());

      POSTION_TYPE point;
      point = (POSTION_TYPE)kdtree_.getPoint(indices[0][0]);
    return point;

  }

  Node<POSTION_TYPE> *extend(const POSTION_TYPE &target)
  {

      POSTION_TYPE source = nearest(target, nullptr);
      if (!source)
      {
          return nullptr;
      }
      
      /*

      //  Get a state that's in the direction of @target from @source. This
      //  should take a step in that direction, but not go all the way unless
      //  the they're really close together.
      T intermediateState;
      if (_isASCEnabled)
      {
          intermediateState = _stateSpace->intermediateState(
              source->state(), target, stepSize(), maxStepSize());
      }
      else
      {
          intermediateState = _stateSpace->intermediateState(
              source->state(), target, stepSize());
      }

      //  Make sure there's actually a direct path from @source to
      //  @intermediateState.  If not, abort
      if (!_stateSpace->transitionValid(source->state(), intermediateState))
      {
          return nullptr;
      }

      // Add a node to the tree for this state
      _nodes.emplace_back(intermediateState, source, _dimensions, _TToArray);
      _kdtree.addPoints(flann::Matrix<double>(
          _nodes.back().coordinates()->data(), 1, _dimensions));
      _nodemap.insert(
          std::pair<T, Node<T> *>(intermediateState, &_nodes.back()));
      return &_nodes.back();*/
  }

private:
    std::shared_ptr<CloudEstimator<POINT_TYPE>> cloud_estimator_;
    flann::Index<flann::L2_Simple<double>> kdtree_;
    Eigen::AlignedBox3d box_;
};

} // namespace RRT

#endif // RRT_TREE_H_

