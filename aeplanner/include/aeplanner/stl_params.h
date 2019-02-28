#ifndef AEPLANNER_STL_PARAMS_H
#define AEPLANNER_STL_PARAMS_H

#include <geometry_msgs/Pose.h>

#include <utility>
#include <map>

namespace aeplanner
{
struct STLParams
{
  double lambda;

  double min_distance;
  double max_distance;

  bool min_distance_active;
  bool max_distance_active;
  bool routers_active;

  std::map<int, std::pair<geometry_msgs::Pose, double>> routers;

  double max_search_distance;
  double step_size;
};
}  // namespace aeplanner

#endif  // AEPLANNER_STL_PARAMS_H
