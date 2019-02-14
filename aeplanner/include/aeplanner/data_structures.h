#ifndef DATA_STRUCTURES_H
#define DATA_STRUCTURES_H

#include <octomap/OcTree.h>
#include <ros/ros.h>

#include <eigen3/Eigen/Dense>

#include <omp.h>
#include <queue>

namespace aeplanner
{
class RRTNode
{
public:
  Eigen::Vector4d state_;
  RRTNode* parent_;
  std::vector<RRTNode*> children_;
  double gain_;
  bool gain_explicitly_calculated_;

  std::shared_ptr<octomap::OcTree> score_ot_;
  RRTNode* score_parent_;
  double score_;
  std::shared_ptr<octomap::OcTree> cost_ot_;
  RRTNode* cost_parent_;
  double cost_;

  RRTNode() : parent_(NULL), gain_(0.0), gain_explicitly_calculated_(false)
  {
  }
  ~RRTNode()
  {
    for (typename std::vector<RRTNode*>::iterator node_it = children_.begin(); node_it != children_.end(); ++node_it)
    {
      delete (*node_it);
      (*node_it) = NULL;
    }
  }

  RRTNode* getCopyOfParentBranch()
  {
    RRTNode* current_node = this;
    RRTNode* current_child_node = NULL;
    RRTNode* new_node;
    RRTNode* new_child_node = NULL;

    while (current_node)
    {
      new_node = new RRTNode();
      new_node->state_ = current_node->state_;
      new_node->gain_ = current_node->gain_;
      new_node->gain_explicitly_calculated_ = current_node->gain_explicitly_calculated_;
      new_node->parent_ = NULL;

      if (new_child_node)
      {
        new_node->children_.push_back(new_child_node);
        new_child_node->parent_ = new_node;
      }

      current_child_node = current_node;
      current_node = current_node->parent_;
      new_child_node = new_node;
    }

    return new_node;
  }

  double getDistanceGain(std::shared_ptr<octomap::OcTree> ot, double ltl_lambda, double min_distance,
                         double max_distance, bool min_distance_active, bool max_distance_active,
                         double max_search_distance, double radius)
  {
    if (!min_distance_active && !max_distance_active)
    {
      return 1;
    }

    double closest_distance = getDistanceToClosestOccupiedBounded(ot, max_search_distance, radius);

    double distance_gain = 0;
    if (min_distance_active && max_distance_active)
    {
      distance_gain = std::min(closest_distance - min_distance, max_distance - closest_distance);
    }
    else if (min_distance_active)
    {
      distance_gain = closest_distance - min_distance;
    }
    else
    {
      distance_gain = max_distance - closest_distance;
    }

    return std::exp(-ltl_lambda * distance_gain);
  }

  double score(std::shared_ptr<octomap::OcTree> ot, double ltl_lambda, double min_distance, double max_distance,
               bool min_distance_active, bool max_distance_active, double max_search_distance, double radius,
               double lambda)
  {
    if (score_ot_ && *ot == *score_ot_ && parent_ == score_parent_)
    {
      return score_;
    }

    score_ot_ = ot;
    score_parent_ = parent_;

    if (!this->parent_)
    {
      score_ = this->gain_;
      return score_;
    }

    double distance_gain = getDistanceGain(ot, ltl_lambda, min_distance, max_distance, min_distance_active,
                                           max_distance_active, max_search_distance, radius);

    score_ = this->parent_->score(ot, ltl_lambda, min_distance, max_distance, min_distance_active, max_distance_active,
                                  max_search_distance, radius, lambda) +
             this->gain_ * exp(-lambda * (this->distance(this->parent_) * std::fmax(distance_gain, 1)));
    return score_;
  }

  double cost(std::shared_ptr<octomap::OcTree> ot, double ltl_lambda, double min_distance, double max_distance,
              bool min_distance_active, bool max_distance_active, double max_search_distance, double radius)
  {
    if (cost_ot_ && *ot == *cost_ot_ && parent_ == cost_parent_)
    {
      return cost_;
    }

    cost_ot_ = ot;
    cost_parent_ = parent_;

    if (!this->parent_)
    {
      cost_ = 0;
      return cost_;
    }

    double distance_gain = getDistanceGain(ot, ltl_lambda, min_distance, max_distance, min_distance_active,
                                           max_distance_active, max_search_distance, radius);

    cost_ = (this->distance(this->parent_) * std::fmax(distance_gain, 1)) +
            this->parent_->cost(ot, ltl_lambda, min_distance, max_distance, min_distance_active, max_distance_active,
                                max_search_distance, radius);
    return cost_;
  }

  double getDistanceToClosestOccupiedBounded(std::shared_ptr<octomap::OcTree> ot, double max_search_distance,
                                             double radius)
  {
    Eigen::Vector3d start;
    if (parent_)
    {
      start[0] = parent_->state_[0];
      start[1] = parent_->state_[1];
      start[2] = parent_->state_[2];
    }
    else
    {
      start[0] = state_[0];
      start[1] = state_[1];
      start[2] = state_[2];
    }
    Eigen::Vector3d end(state_[0], state_[1], state_[2]);

    Eigen::Vector3d min(std::min(start[0] - radius, end[0] - max_search_distance),
                        std::min(start[1] - radius, end[1] - max_search_distance), std::min(start[2], end[2]) - radius);
    Eigen::Vector3d max(std::max(start[0] + radius, end[0] + max_search_distance),
                        std::max(start[1] + radius, end[1] + max_search_distance), std::max(start[2], end[2]) + radius);

    double lsq = (end - start).squaredNorm();
    double rsq = max_search_distance * max_search_distance;

    std::set<std::tuple<double, double, double>> closed_set;
    std::vector<std::priority_queue<std::tuple<double, double, double, double>>> open_set(4);

    std::vector<octomap::point3d> line_points;
    ot->computeRay(octomap::point3d(start[0], start[1], start[2]), octomap::point3d(end[0], end[1], end[2]),
                   line_points);

    for (octomap::point3d point : line_points)
    {
      octomap::OcTreeKey key = ot->coordToKey(point, ot->getTreeDepth() - (open_set.size() - 1));
      // open_set[0].emplace(key[0], key[1], key[2]);
      open_set[0].emplace(0.0, point.x(), point.y(), point.z());
    }

    for (size_t i = 0; i < open_set.size() - 2; ++i)
    {
      ROS_INFO_STREAM("i: " << i);
      double res = ot->getResolution() * std::pow((open_set.size() - 1) - i, 2.0);
      double next_res = ot->getResolution() * std::pow((open_set.size() - 1) - (i + 1), 2.0);

      closed_set.clear();
      while (!open_set[i].empty())
      {
        std::tuple<double, double, double, double> current = open_set[i].top();
        Eigen::Vector3d point(std::get<1>(current), std::get<2>(current), std::get<3>(current));
        open_set[i].pop();
        if (!closed_set.emplace(point[0], point[1], point[2]).second)
        {
          continue;
        }

        if (point[2] >= min[2] && point[2] <= max[2])
        {
          double distance = computeDistance(point, start, end);

          if (distance > max_search_distance)
          {
            continue;
          }

          if (open_set.size() - 1 > i && !open_set[i + 1].empty())
          {
            if (distance > std::get<0>(open_set[i + 1].top()))
            {
              continue;
            }
          }

          octomap::OcTreeNode* v =
              ot->search(point[0], point[1], point[2], ot->getTreeDepth() - ((open_set.size() - 1) - i));

          if (v != NULL && v->getLogOdds() > 0)
          {
            if (i + 1 < open_set.size() - 2)
            {
              // Add children
              for (double x = -next_res; x <= next_res; x += next_res)
              {
                for (double y = -next_res; y <= next_res; y += next_res)
                {
                  for (double z = -next_res; z <= next_res; z += next_res)
                  {
                    open_set[i + 1].emplace(computeDistance(point + Eigen::Vector3d(x, y, z), start, end), point[0] + x,
                                            point[1] + y, point[2] + z);
                  }
                }
              }
            }
            else
            {
              return distance;
            }
          }

          if (open_set.size() >= 2 && open_set[1].empty())
          {
            // Add children
            for (double x = -res; x <= res; x += res)
            {
              for (double y = -res; y <= res; y += res)
              {
                for (double z = -res; z <= res; z += res)
                {
                  if (x != 0 || y != 0 || z != 0)
                  {
                    open_set[i].emplace(computeDistance(point + Eigen::Vector3d(x, y, z), start, end), point[0] + x,
                                        point[1] + y, point[2] + z);
                  }
                }
              }
            }
          }
        }
      }
    }

    return 10000000;
  }

  double computeDistance(Eigen::Vector3d point, Eigen::Vector3d start, Eigen::Vector3d end)
  {
    Eigen::Vector3d d = (end - start) / (end - start).norm();
    Eigen::Vector3d v = point - start;
    double t = v.dot(d);
    Eigen::Vector3d P = start + t * d;
    return (point - P).norm();
  }

  double distance(RRTNode* other)
  {
    Eigen::Vector3d p3(this->state_[0], this->state_[1], this->state_[2]);
    Eigen::Vector3d q3(other->state_[0], other->state_[1], other->state_[2]);
    return (p3 - q3).norm();
  }

  //-----------------------------------------------------------------------------
  // Name: CylTest_CapsFirst
  // Orig: Greg James - gjames@NVIDIA.com
  // Lisc: Free code - no warranty & no money back.  Use it all you want
  // Desc:
  //    This function tests if the 3D point 'pt' lies within an arbitrarily
  // oriented cylinder.  The cylinder is defined by an axis from 'pt1' to 'pt2',
  // the axis having a length squared of 'lsq' (pre-compute for each cylinder
  // to avoid repeated work!), and radius squared of 'rsq'.
  //    The function tests against the end caps first, which is cheap -> only
  // a single dot product to test against the parallel cylinder caps.  If the
  // point is within these, more work is done to find the distance of the point
  // from the cylinder axis.
  //    Fancy Math (TM) makes the whole test possible with only two dot-products
  // a subtract, and two multiplies.  For clarity, the 2nd mult is kept as a
  // divide.  It might be faster to change this to a mult by also passing in
  // 1/lengthsq and using that instead.
  //    Elminiate the first 3 subtracts by specifying the cylinder as a base
  // point on one end cap and a vector to the other end cap (pass in {dx,dy,dz}
  // instead of 'pt2' ).
  //
  //    The dot product is constant along a plane perpendicular to a vector.
  //    The magnitude of the cross product divided by one vector length is
  // constant along a cylinder surface defined by the other vector as axis.
  //
  // Return:  -1.0 if point is outside the cylinder
  // Return:  distance squared from cylinder axis if point is inside.
  //
  //-----------------------------------------------------------------------------
  float CylTestCapsFirst(const Eigen::Vector3d& pt1, const Eigen::Vector3d& pt2, float lsq, float rsq,
                         const Eigen::Vector3d& pt)
  {
    float dx, dy, dz;     // vector d  from line segment point 1 to point 2
    float pdx, pdy, pdz;  // vector pd from point 1 to test point
    float dot, dsq;

    dx = pt2[0] - pt1[0];  // translate so pt1 is origin.  Make vector from
    dy = pt2[1] - pt1[1];  // pt1 to pt2.  Need for this is easily eliminated
    dz = pt2[2] - pt1[2];

    pdx = pt[0] - pt1[0];  // vector from pt1 to test point.
    pdy = pt[1] - pt1[1];
    pdz = pt[2] - pt1[2];

    // Dot the d and pd vectors to see if point lies behind the
    // cylinder cap at pt1.x, pt1.y, pt1.z

    dot = pdx * dx + pdy * dy + pdz * dz;

    // If dot is less than zero the point is behind the pt1 cap.
    // If greater than the cylinder axis line segment length squared
    // then the point is outside the other end cap at pt2.

    if (dot < 0.0f || dot > lsq)
      return (-1.0f);
    else
    {
      // Point lies within the parallel caps, so find
      // distance squared from point to line, using the fact that sin^2 + cos^2 = 1
      // the dot = cos() * |d||pd|, and cross*cross = sin^2 * |d|^2 * |pd|^2
      // Carefull: '*' means mult for scalars and dotproduct for vectors
      // In short, where dist is pt distance to cyl axis:
      // dist = sin( pd to d ) * |pd|
      // distsq = dsq = (1 - cos^2( pd to d)) * |pd|^2
      // dsq = ( 1 - (pd * d)^2 / (|pd|^2 * |d|^2) ) * |pd|^2
      // dsq = pd * pd - dot * dot / lengthsq
      //  where lengthsq is d*d or |d|^2 that is passed into this function

      // distance squared to the cylinder axis:

      dsq = (pdx * pdx + pdy * pdy + pdz * pdz) - dot * dot / lsq;

      if (dsq > rsq)
        return (-1.0f);
      else
        return (dsq);  // return distance squared to axis
    }
  }
};
}  // namespace aeplanner

#endif
