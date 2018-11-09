#pragma once

#include <deque>
#include <flann/algorithms/dist.h>
#include <flann/algorithms/kdtree_single_index.h>
#include <flann/flann.hpp>
#include <functional>
#include <list>
#include <memory>
#include <stdexcept>
#include <stdlib.h>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <vector>


#include "node.h"
#include "StateSpace.h"


namespace RRT {


/**
 * An RRT tree searches a state space by randomly filling it in and
 * connecting points to form a branching tree.  Once a branch of the tree
 * reaches the goal and satisifes all of the constraints, a solution is
 * found and returned.
 *
 * This provides a base class for RRT trees.  Because many parts of an RRT are
 * implementation-/domain-specific, several key functionalities are placed in
 * callbacks (C++ lambdas), which must be supplied by the user of this class.
 *
 * If adaptive stepsize control (ASC) is enabled, then the stepsize for
 * extending new nodes from the tree will be dynamically updated depending on
 * how close the nearest obstacle is. If there are no nearby obstacles, then the
 * stepsize will be extended in order to safely cover more ground. If there are
 * nearby obstacles, then the stepsize will shrink so that the RRT can take more
 * precise steps.
 *
 * USAGE:
 * 1) Create a new Tree with the appropriate StateSpace
 *    RRT::Tree<My2dPoint> tree(stateSpace, hashT, arrayToT, TToArray);
 *
 *    hashT is a function pointer to a hash function for T
 *    arrayToT is an optional function pointer to convert from an array of
 *doubles to T
 *    TToArray is an optional function pointer to convert from T to an array of
 *doubles
 *    arrayToT and TToArray must be provided if T does not possess that
 *functionality
 *
 * 2) Set the start and goal states
 *    tree->setStartState(s);
 *    tree->setGoalState(g);
 *
 * 3) (Optional) If adaptive stepsize control is enabled:
 *    stateSpace_->setMaxStepSize sets the maximum stepsize the tree can take
 * for any step.
 *
 * 4) Run the RRT algorithm!  This can be done in one of two ways:
 *    Option 1) Call the run() method - it will grow the tree
 *              until it finds a solution or runs out of iterations.
 *
 *    Option 2) Call grow() repeatedly
 *
 *    Either way works fine, just choose whatever works best for your
 *    application.
 *
 * 5) Use getPath() to get the series of states that make up the solution
 *
 * @param T The type that represents a state within the state-space that
 * the tree is searching.  This could be a 2D Point or something else,
 * but will generally be some sort of vector.
 */
template <typename T>
class Tree {
public:
    Tree(const Tree&) = delete;
    Tree& operator=(const Tree&) = delete;
    Tree(std::shared_ptr<StateSpace<T>> stateSpace,
         std::function<size_t(T)> hashT, int dimensions)
        : nodemap_(20, hashT),
          //near_states_(20,hashT),
          //near_states_active_(20,hashT),
          dimensions_(dimensions),
          kdtree_(flann::KDTreeSingleIndexParams())
            {
        stateSpace_ = stateSpace;

        //  default values
        setStepSize(0.1);
        setMaxIterations(1000);
        setGoalMaxDist(0.1);
    }

    StateSpace<T>& stateSpace() { return *stateSpace_; }
    const StateSpace<T>& stateSpace() const { return *stateSpace_; }

    /**
     * The maximum number of random states in the state-space that we will
     * try before giving up on finding a path to the goal.
     */
    int maxIterations() const { return maxIterations_; }
    void setMaxIterations(int itr) { maxIterations_ = itr; }


    double stepSize() const { return stepSize_; }
    void setStepSize(double stepSize) { stepSize_ = stepSize; }

    /**
     * @brief How close we have to get to the goal in order to consider it
     *     reached.
     * @details The RRT will continue to run unti we're within @goalMaxDist of
     *     the goal state.
     * reached.
     * @details The RRT will continue to run unti we're within @goalMaxDist of
     * the
     * goal state.
     */
    double goalMaxDist() const { return goalMaxDist_; }
    void setGoalMaxDist(double maxDist) { goalMaxDist_ = maxDist; }

    /**
     * Executes the RRT algorithm with the given start state.
     *
     * @return a bool indicating whether or not it found a path to the goal
     */
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

    /**
     * Removes nodes from nodes_ and nodemap_ so it can be run() again.
     */
    void reset(bool eraseRoot = true)
    {
        //near_states_.clear();
        //near_states_active_.clear();

        kdtree_ = flann::Index<flann::L2_Simple<double>>(
            flann::KDTreeSingleIndexParams());
        nodes_.clear();
        nodemap_.clear();

    }

    /**
     * Picks a random state and attempts to extend the tree towards it.
     * This is called at each iteration of the run() method.
     */
    Node<T> *grow()
    {
        //  extend towards goal, waypoint, or random state depending on the
        //  biases and a random number
        return extend(stateSpace_->randomState());
        //T rand_state;
        //getRandomStateInNearbySet(rand_state);
        //return extend(rand_state);
    }
/*
    bool findStateCloseTree(const T& state)
    {
        T rand_state;
        double dist;
        int c = 0;
        while (c++)
        {
            rand_state = stateSpace_->randomState();
            nearest(rand_state, &dist);
            if (dist < stepSize_*20)
            {
                break;
            }
            if(c>)
        }
        return rand_state;
    }*/
    /**
     * Find the node int the tree closest to @state.  Pass in a double pointer
     * as the second argument to get the distance that the node is away from
     * @state. This method searches a k-d tree of the points to determine
     */
    Node<T>* nearest(const T& state, double* distanceOut = nullptr) {

        // k-NN search (O(log(N)))
        flann::Matrix<double> query;
            query = flann::Matrix<double>((double*)&state, 1,
                                          sizeof(state) / sizeof(0.0));
        std::vector<int> i(query.rows);
        flann::Matrix<int> indices(i.data(), query.rows, 1);
        std::vector<double> d(query.rows);
        flann::Matrix<double> dists(d.data(), query.rows, 1);

        kdtree_.knnSearch(query, indices, dists, 1, flann::SearchParams());

        T point;
        point = (T)kdtree_.getPoint(indices[0][0]);

        if (distanceOut){
            *distanceOut = stateSpace_->distance(state, point);
        }

        return nodemap_[point];
    }
    /*
    bool getRandomStateInNearbySet(T &state){
        if (near_states_active_.empty())
            return false;
        auto item = near_states_active_.begin();
        int rand_id = rand() % near_states_active_.size();
        std::advance(item, rand_id);
        state = *item;
        near_states_active_.erase(item);
        return true;
    }
    */
    void addNode(const T &target, Node<T> *parent)
    {
        nodes_.emplace_back(target, parent, dimensions_);
        kdtree_.addPoints(flann::Matrix<double>(
            nodes_.back().coordinates()->data(), 1, dimensions_));
        nodemap_.insert(
            std::pair<T, Node<T> *>(target, &nodes_.back()));
        /*
        auto node = &this->nodes_.back();
        auto states = this->stateSpace_->radiusRandomState(node->state(), 5);

        for (T s : states)
        {
            if(near_states_.count(s) == 0)
            {
                near_states_.insert(s);
                near_states_active_.insert(s);
            }
        }
        */
    }

    /**
     * Grow the tree in the direction of @state
     *
     * @return the new tree Node (may be nullptr if we hit Obstacles)
     * @param target The point to extend the tree to
     * @param source The Node to connect from.  If source == nullptr, then
     *             the closest tree point is used
     */
    virtual Node<T>* extend(const T& target, Node<T>* source = nullptr) {
        //  if we weren't given a source point, try to find a close node
        //if(nodemap_.count(target) != 0)
        //    return nullptr;
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
        addNode(intermediateState, source);
        return &nodes_.back();
    }

    /**
     * Get the path from the receiver's root point to the dest point
     *
     * @param callback The lambda to call for each state in the path
     * @param dest The node in the tree to get the path for. If nullptr, will
     *     use the the last point added to the @nodes_ vector. If run() was just
     *     called successfully, this node will be the one last created that is
     *     closest to the goal.
     * @param reverse if true, the states will be sent from @dest to the tree's
     *     root
     */
    void getPath(std::function<void(const T& stateI)> callback,
                 const Node<T>* dest = nullptr, bool reverse = false) const {
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
        }
    }

    /**
     * The same as the first getPath() method, but appends the states to a given
     * output vector rather than executing a callback.
     *
     * @param vectorOut The vector to append the states along the path
     */
    void getPath(std::vector<T>* vectorOut, const Node<T>* dest = nullptr,
                 bool reverse = false) const {
        getPath([&](const T& stateI) { vectorOut->push_back(stateI); }, dest,
                reverse);
    }

    /**
     * The same as the first getPath() method, but returns the vector of states
     * instead of executing a callback.
     */
    std::vector<T> getPath(const Node<T>* dest = nullptr,
                           bool reverse = false) const {
        std::vector<T> path;
        getPath(&path, dest, reverse);
        return path;
    }

    /**
     * @return The root node or nullptr if none exists
     */
    const Node<T>* rootNode() const {
        if (nodes_.empty()) return nullptr;

        return &nodes_.front();
    }

    /**
     * @return The most recent Node added to the tree
     */
    const Node<T>* lastNode() const {
        if (nodes_.empty()) return nullptr;

        return &nodes_.back();
    }

    /**
     * All the nodes
     */
    const std::deque<Node<T>>& allNodes() const { return nodes_; }

    /**
     * @brief The start state for this tree
     */
    const T& startState() const {
        if (nodes_.empty())
            throw std::logic_error("No start state specified for RRT::Tree");
        else
            return rootNode()->state();
    }
    void setStartState(const T& startState) {
        reset();

        //  create root node from provided start state
        kdtree_.buildIndex(flann::Matrix<double>(
            (double*)&(startState), 1, dimensions_));
        addNode(startState, nullptr);

    
    }

    /**
     * @brief The goal this tree is trying to reach.
     */
    const T& goalState() const { return goalState_; }
    void setGoalState(const T& goalState) { goalState_ = goalState; }

protected:
    /**
     * A list of all Node objects in the tree.
     */
    std::deque<Node<T>> nodes_{};

    std::unordered_map<T, Node<T>*, std::function<size_t(T)>> nodemap_;

    T goalState_;

    const int dimensions_;

    int maxIterations_;

    /// previously grown tree

    double goalMaxDist_;

    double stepSize_;

    flann::Index<flann::L2_Simple<double>> kdtree_;

    std::shared_ptr<StateSpace<T>> stateSpace_;

    //std::unordered_set<T, std::function<size_t(T)>> near_states_;
    //std::unordered_set<T, std::function<size_t(T)>> near_states_active_;
};


}  // namespace RRT
