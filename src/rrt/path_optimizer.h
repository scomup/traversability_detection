#pragma once

#include <limits.h>
#include "node.h"

namespace RRT {
/**
 * @brief Bi-directional RRT
 * @details It is often preferable to use two RRTs when searching the state
 *     space with one rooted at the source and one rooted at the goal.  When the
 *     two trees intersect, a solution has been found.
 */

template <typename T>
class PathOptimizer
{
  public:
    PathOptimizer(std::vector<T>& path_)
    {
    }


  private:
  std::vector<T> path_;
};

} // namespace RRT
