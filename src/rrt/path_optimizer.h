#pragma once

#include <random>
#include <limits.h>
#include "node.h"
#include "Tree.h"

namespace RRT
{
/**
 * @brief Bi-directional RRT
 * @details It is often preferable to use two RRTs when searching the state
 *     space with one rooted at the source and one rooted at the goal.  When the
 *     two trees intersect, a solution has been found.
 */

template <typename T>
class PathOptimizer : public Tree<Eigen::Vector3d>
{
public:
  PathOptimizer(const PathOptimizer &) = delete;
  PathOptimizer &operator=(const PathOptimizer &) = delete;
  PathOptimizer(std::shared_ptr<StateSpace<T>> stateSpace,
                std::function<size_t(T)> hashT, int dimensions)
  : Tree(stateSpace, stateSpace, dimensions)
  //  dimensions_(dimensions),
  //  kdtree_(flann::KDTreeSingleIndexParams())
  {
    //  default values
    //setStepSize(0.1);
    //setMaxIterations(1000);
    //setGoalMaxDist(0.1);
  }

  void SetInitPath(std::vector<Eigen::Vector3d> &path)
  {
    setStartState(path.front());
    setGoalState(path.back());
    for (size_t i = 1; i < path.size(); i++)
    {
      nodes_.emplace_back(path[i], path[i-1], dimensions_);
      kdtree_.addPoints(flann::Matrix<double>(
          nodes_.back().coordinates()->data(), 1, dimensions_));
      nodemap_.insert(
          std::pair<T, Node<T> *>(path[i], &nodes_.back()));
    }
  }
  /*
    bool run() {
        //  grow the tree until we find the goal or run out of iterations
        for (int i = 0; i < maxIterations_; i++) {
            Node<T>* newNode = grow();

            if (newNode &&
                stateSpace_->distance(newNode->state(), goalState_) <
                    goalMaxDist_)
                return true;
        }

        //  we hit our iteration limit and didn't reach the goal :(
        return false;
    }
*/

  
    Node<T> *grow()
    {
      
      int idx = rand() % nodemap_.size();
      auto random_it = std::next(std::begin(nodemap_),idx);
      auto sample = random_it->first;
      return extend(stateSpace_->radiusRandomState(sample, samp_radius_));
      
      
    }

    std::vector<Node<T>*> radiusNear(const T& state, double radius) {
        Node<T>* best = nullptr;

        // k-NN search (O(log(N)))
        flann::Matrix<double> query;
            query = flann::Matrix<double>((double*)&state, 1,
                                          sizeof(state) / sizeof(0.0));
        std::vector<int> i(query.rows);
        flann::Matrix<int> indices(i.data(), query.rows, 1);
        std::vector<double> d(query.rows);
        flann::Matrix<double> dists(d.data(), query.rows, 1);

        kdtree_.radiusSearch(query, indices, dists, radius, flann::SearchParams());

        std::vector<Node<T> *> nodes;
        for (int i = 0; i < indices.rows(); i++)
        {
          T point;
          point = (T)kdtree_.getPoint(indices[i][0]);
          nodes.push_back(point);
        }

        return nodes;
    }
  
    virtual Node<T>* extend(const T& target, Node<T>* source = nullptr) {
        //  if we weren't given a source point, try to find a close node
        if (!source) {
            source = nearest(target, nullptr);
            if (!source) {
                return nullptr;
            }
        }
        //  Get a state that's in the direction of @target from @source. This
        //  should take a step in that direction, but not go all the way unless
        //  the they're really close together.
        T intermediateState;
        intermediateState = stateSpace_->intermediateState(
            source->state(), target, stepSize());

        //  Make sure there's actually a direct path from @source to
        //  @intermediateState.  If not, abort
        if (!stateSpace_->transitionValid(source->state(), intermediateState)) {
            return nullptr;
        }

        // Add a node to the tree for this state
        nodes_.emplace_back(intermediateState, source, dimensions_);
        kdtree_.addPoints(flann::Matrix<double>(
            nodes_.back().coordinates()->data(), 1, dimensions_));
        nodemap_.insert(
            std::pair<T, Node<T>*>(intermediateState, &nodes_.back()));

        auto near_nodes = radiusNear(target, samp_radius_);

        for (auto &node : near_nodes)
        {
          if (stateSpace_->transitionValid(target, node->state()) )
          {
            //double target  node->state()
            //node->cost() + target
          }
        }

        return &nodes_.back();
    }

  protected:
  double samp_radius_ = 5.;

}; // namespace RRT

} // namespace RRT
