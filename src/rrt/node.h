#ifndef RRT_NODE_H_
#define RRT_NODE_H_

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
#include <vector>

namespace RRT
{

template <typename T>
class Node
{
public:
  Node(const T &state, Node<T> *parent = nullptr)
      : parent_(parent),
        postion_(state)
  {
    if (parent_)
    {
      parent_->children_.push_back(this);
    }
  }

  const Node<T> *parent() const { return parent_; }

  int depth() const
  {
    int n = 0;
    for (Node<T> *ancestor = parent_; ancestor != nullptr;
         ancestor = ancestor->parent_)
    {
      n++;
    }
    return n;
  }

  const T &postion() const { return postion_; }

private:
  T postion_;
  std::list<Node<T> *> children_;
  Node<T> *parent_;
};
} // namespace RRT

#endif // RRT_NODE_H_

