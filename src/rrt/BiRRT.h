#pragma once

#include <limits.h>
#include "Tree.h"


namespace RRT
{
/**
 * @brief Bi-directional RRT
 * @details It is often preferable to use two RRTs when searching the state
 *     space with one rooted at the source and one rooted at the goal.  When the
 *     two trees intersect, a solution has been found.
 */
class PlannerHandle;

template <typename T>
class BiRRT
{

    friend class PlannerHandle;

  public:
    BiRRT(std::shared_ptr<StateSpace<T>> stateSpace,
          std::function<size_t(T)> hash, int dimensions)
        : startTree_(stateSpace, hash, dimensions),
          goalTree_(stateSpace, hash, dimensions)
    {
        minIterations_ = 0;
        reset();
    }

    void reset()
    {
        startTree_.reset();
        goalTree_.reset();

        iterationCount_ = 0;

        startSolutionNode_ = nullptr;
        goalSolutionNode_ = nullptr;
        solutionLength_ = INT_MAX;
    }

    const Tree<T> &startTree() const { return startTree_; }
    const Tree<T> &goalTree() const { return goalTree_; }

    double goalBias() const { return startTree_.goalBias(); }
    void setGoalBias(double goalBias)
    {
        startTree_.setGoalBias(goalBias);
        goalTree_.setGoalBias(goalBias);
    }

    int maxIterations() const { return startTree_.maxIterations(); }
    void setMaxIterations(int itr)
    {
        startTree_.setMaxIterations(itr);
        goalTree_.setMaxIterations(itr);
    }

    /**
     * The minimum number of iterations to run.
     *
     * At the default value of zero, the rrt will return the first path it
     * finds. Setting this to a higher value can allow the tree to search for
     * longer in order to find a better path.
     */
    int minIterations() const { return minIterations_; }
    void setMinIterations(int itr) { minIterations_ = itr; }


    double stepSize() const { return startTree_.stepSize(); }
    void setStepSize(double stepSize)
    {
        startTree_.setStepSize(stepSize);
        goalTree_.setStepSize(stepSize);
    }


    double goalMaxDist() const { return startTree_.goalMaxDist(); }
    void setGoalMaxDist(double maxDist)
    {
        startTree_.setGoalMaxDist(maxDist);
        goalTree_.setGoalMaxDist(maxDist);
    }

    /**
     * @brief Get the shortest path from the start to the goal
     */
    std::vector<T> getPath()
    {
        std::vector<T> path;
        startTree_.getPath(&path, startSolutionNode_);
        startTree_.getPath(&path, goalSolutionNode_, true);
        return path;
    }

    /**
     * @brief
     * @details Attempts to add a new node to each of the two trees.  If
     * a new solution is found that is shorter than any previous solution, we
     * store
     * it instead.
     */
    void grow()
    {
        int depth;
        const Node<T> *otherNode;

        Node<T> *newStartNode = startTree_.grow();
        if (newStartNode)
        {
            otherNode = findBestPath(newStartNode->state(), goalTree_, &depth);

            if (otherNode && depth + newStartNode->depth() < solutionLength_ &&
                goalTree_.stateSpace().transitionValid(newStartNode->state(),
                                                       otherNode->state()))
            {
                startSolutionNode_ = newStartNode;
                goalSolutionNode_ = otherNode;
                solutionLength_ = newStartNode->depth() + depth;
            }
        }

        Node<T> *newGoalNode = goalTree_.grow();
        if (newGoalNode)
        {
            otherNode = findBestPath(newGoalNode->state(), startTree_, &depth);
            if (otherNode && depth + newGoalNode->depth() < solutionLength_ &&
                goalTree_.stateSpace().transitionValid(newGoalNode->state(),
                                                       otherNode->state()))
            {
                startSolutionNode_ = otherNode;
                goalSolutionNode_ = newGoalNode;
                solutionLength_ = newGoalNode->depth() + depth;
            }
        }

        ++iterationCount_;
    }

    /**
     * @brief Grows the trees until we find a solution or run out of iterations.
     * @return true if a solution is found
     */
    bool run()
    {
        static bool running = false;
        static int i = 0;
        if (running == true && step_run == false)
            return false;
        while (i < startTree_.maxIterations())
        {
            running = true;
            grow();
            //std::cout << i << std::endl;
            if (startSolutionNode_ != nullptr && i >= minIterations())
            {
                running = false;
                i = 0;
                return true;
            }
            i++;
        }
        running = false;
        i = 0;
        return false;
    }

    bool step_run()
    {
        static int i = 0;

        if (startSolutionNode_ || i >= startTree_.maxIterations())
        {
            i = 0;
            return true;
        }
        grow();
        i++;
        return false;
    }

    void setStartState(const T &start)
    {
        startTree_.setStartState(start);
        goalTree_.setGoalState(start);
    }
    const T &startState() const { return startTree_.startState(); }

    void setGoalState(const T &goal)
    {
        startTree_.setGoalState(goal);
        goalTree_.setStartState(goal);
    }
    const T &goalState() const { return startTree_.goalState(); }

    const Node<T> *startSolutionNode() { return startSolutionNode_; }

    const Node<T> *goalSolutionNode() { return goalSolutionNode_; }

    int iterationCount() const { return iterationCount_; }

  protected:
    const Node<T> *findBestPath(const T &targetState, Tree<T> &treeToSearch,
                                 int *depthOut) const
    {
        const Node<T> *node = treeToSearch.nearest(targetState);
        const Node<T> *bestNode = nullptr;
        int depth = INT_MAX;

        while (node != nullptr)
        {
            double dist = startTree_.stateSpace().distance(node->state(), targetState);

            if (dist < goalMaxDist() && node->depth() < depth &&
                treeToSearch.stateSpace().transitionValid(targetState, node->state()))
            {
                bestNode = node;
                depth = node->depth();
            }
            node = node->parent();
        }

        if (depthOut)
            *depthOut = depth;

        return bestNode;
    }

  private:
    Tree<T> startTree_;
    Tree<T> goalTree_;

    int iterationCount_;
    int minIterations_;

    int solutionLength_;
    const Node<T> *startSolutionNode_, *goalSolutionNode_;
};

} // namespace RRT
