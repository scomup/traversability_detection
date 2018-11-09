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
class PathOptimizer : public Tree<T>
{
  public:
    PathOptimizer(const PathOptimizer &) = delete;
    PathOptimizer &operator=(const PathOptimizer &) = delete;
    PathOptimizer(std::shared_ptr<StateSpace<T>> stateSpace,
                  std::function<size_t(T)> hashT, int dimensions)
        : Tree<T>(stateSpace, hashT, dimensions)
    {
        //  default values
        //setStepSize(0.1);
        //setMaxIterations(1000);
        //setGoalMaxDist(0.1);
    }

    void SetInitPath(std::vector<T> &path)
    {

        this->setStartState(path.front());
        this->setGoalState(path.back());

        for (size_t i = 1; i < path.size(); i++)
        {
            this->nodes_.emplace_back(path[i], &(this->nodes_.back()), this->dimensions_);
            std::cout << this->nodes_.back().coordinates()->data()[0] << ","
                      << this->nodes_.back().coordinates()->data()[1] << ","
                      << this->nodes_.back().coordinates()->data()[2]
                      << std::endl;
            this->kdtree_.addPoints(flann::Matrix<double>(
                this->nodes_.back().coordinates()->data(), 1, this->dimensions_));

            this->nodemap_.insert(
                std::pair<T, Node<T> *>(path[i], &this->nodes_.back()));
        }

                                                             
        /*
        flann::Matrix<double> query;
        query = flann::Matrix<double>((double *)&path[0], 1,
                                      sizeof(path[0]) / sizeof(0.0));
        std::vector<int> i(10);
        flann::Matrix<int> indices(i.data(), query.rows, 1);
        std::vector<double> d(10);
        flann::Matrix<double> dists(d.data(), query.rows, 1);
        int s = this->kdtree_.size();
        int n = this->kdtree_.radiusSearch(query, indices, dists, 10000000., flann::SearchParams(-1));
        std::cout<<"tot:"<<s<<std::endl<<"sphere:"<<n<<std::endl;
        */
        
        
    }

    bool step_run()
    {
        static int i = 0;

        if (i >= this->maxIterations_)
        {
            i = 0;
            return true;
        }
        grow();
        i++;
        return false;
    }


    bool run()
    {
        //  grow the tree until we find the goal or run out of iterations
        for (int i = 0; i < this->maxIterations_; i++)
        {
            Node<T> *newNode = grow();
            if(newNode != nullptr){
                //std::this_thread::sleep_for(std::chrono::seconds(1));

            }

            if (newNode &&
                this->stateSpace_->distance(newNode->state(), this->goalState_) <
                    this->goalMaxDist_)
                return true;
        }

        //  we hit our iteration limit and didn't reach the goal :(
        return false;
    }

    Eigen::Vector3d FindStateCloseTree() 
    {
        Eigen::Vector3d rand_state;
        double dist;
        while (true)
        {
            rand_state = this->stateSpace_->randomState();
            Tree<Eigen::Vector3d>::nearest(rand_state, &dist);
            if (dist < samp_radius_)
            {
                break;
            }
        }
        return rand_state;
    }

    Node<T> *grow()
    {
        auto rand_point = FindStateCloseTree();
        return extend(rand_point);
    }

    std::vector<Node<T>*> radiusNear(const T &state, double radius)
    {
        std::vector<double> query
         = {state(0),state(1),state(2)};
        std::vector<std::vector<int>> indices(1);
        std::vector<std::vector<double>> dists(1);

        int neighbors_in_radius = this->kdtree_.radiusSearch(::flann::Matrix<double>(&query[0], 1, 3),
                                                             indices,
                                                             dists,
                                                             static_cast<double>(radius*radius),
                                                             flann::SearchParams(-1));
        std::vector<Node<T>*> nodes;
        for (int i = 0; i < neighbors_in_radius; i++)
        {
            T point = (T)this->kdtree_.getPoint(indices[0][i]);
            nodes.push_back(this->nodemap_[point]);
        }
        return nodes;
    }

    virtual Node<T> *extend(const T &target, Node<T> *source = nullptr)
    {
        //  if we weren't given a source point, try to find a close node

        //  Get a state that's in the direction of @target from @source. This
        //  should take a step in that direction, but not go all the way unless
        //  the they're really close together.
        //T intermediateState;
        //intermediateState = this->stateSpace_->intermediateState(
        //    source->state(), target, this->stepSize());

        //  Make sure there's actually a direct path from @source to
        //  @intermediateState.  If not, abort

        auto all_near_nodes = radiusNear(target, samp_radius_);
        double min_cost = 1000;
        for (auto &node : all_near_nodes)
        {
            auto diff = target - node->state();
            auto dist = diff.norm();
            if (!this->stateSpace_->transitionValid(node->state(), target))
                continue;
            if(node->cost() < min_cost){
                min_cost = node->cost();
                source = node;
            }
        }
        if(source == nullptr)
            return nullptr;

        this->addNode(target, source);
        // Add a node to the tree for this state

        auto new_node = &this->nodes_.back();

        for (auto &node : all_near_nodes)
        {
            if (this->stateSpace_->transitionValid(new_node->state(), node->state()))
            {
                
                auto diff = new_node->state() - node->state();
                double new_cost = new_node->cost() + diff.norm();
                
                if (new_cost < node->cost())
                {
                    //node->setParent(&new_node);
                    node->changeParent(new_node);
                }

            
                //double target  node->state()
                //node->cost() + target
            }
        }

        return &this->nodes_.back();
    }
    std::vector<Eigen::Vector3d> getPath() {
        const Node<Eigen::Vector3d>*  goalnode = this->nodemap_[this->goalState_];
        
        std::vector<T> path;
        if(goalnode)
        std::cout<<goalnode->cost()<<std::endl;
        this->Tree<Eigen::Vector3d>::getPath(&path, goalnode, false);
        return path;
        /*
        const Node<T>* node = (dest != nullptr) ? dest : lastNode();
        if (reverse) {
            while (node) {
                callback(node->state());
                node = node->parent();
            }
        } else {
            // collect states in list in leaf -> root order
            std::vector<const Node<T>*> nodes;
            while (node) {
                nodes.push_back(node);
                node = node->parent();
            }

            // pass them one-by-one to the callback, reversing the order so
            // that the callback is called with the start point first and the
            // dest point last
            for (auto itr = nodes.rbegin(); itr != nodes.rend(); itr++) {
                callback((*itr)->state());
            }
        }*/
    }

  protected:
    double samp_radius_ = 5.;

}; // namespace RRT

} // namespace RRT
