#pragma once

#include <memory>
#include <vector>



namespace RRT {
/**
 * Base class for an RRT tree node.
 *
 * @param T The datatype representing the state in the space the RRT
 * will be searching.
 */
template <typename T>
class Node {
public:
    Node(const T& state, Node<T>* parent = nullptr, int dimensions = 3)
        :  vec_(dimensions), state_(state), parent_(parent), cost_(0) {
        if (parent_)
        {
            parent_->children_.push_back(this);
        }
        for (int i = 0; i < dimensions; i++)
        {
            vec_[i] = state[i];
        }
    }

    const Node<T>* parent() const { return parent_; }

    /**
     * Gets the number of ancestors (parent, parent's parent, etc) that
     * the node has.
     * Returns 0 if it doesn't have a parent.
     */
    int depth() const {
        int n = 0;
        for (Node<T>* ancestor = parent_; ancestor != nullptr;
             ancestor = ancestor->parent_) {
            n++;
        }
        return n;
    }

    /**
     * The @state property is the point in the state-space that this
     * Node represents.  Generally this is a vector (could be 2d, 3d, etc)
     */
    const T& state() const { return state_; }

    std::vector<double>* coordinates() { return &vec_; }

private:
    std::vector<double> vec_;
    T state_;
    std::list<Node<T>*> children_;
    Node<T>* parent_;
    double cost_;
};

}  // namespace RRT
