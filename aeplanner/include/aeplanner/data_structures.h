#ifndef DATA_STRUCTURES_H
#define DATA_STRUCTURES_H

#include <octomap/OcTree.h>
#include <ros/ros.h>

#include <eigen3/Eigen/Dense>

#include <omp.h>
#include <queue>

#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>

namespace aeplanner
{
// Rtree
typedef boost::geometry::model::point<double, 3, boost::geometry::cs::cartesian> point;
typedef boost::geometry::model::box<point> box;
typedef point value;

class RRTNode
{
public:
  Eigen::Vector4d state_;
  RRTNode* parent_;
  std::vector<RRTNode*> children_;
  double gain_;
  bool gain_explicitly_calculated_;

  std::shared_ptr<boost::geometry::index::rtree<value, boost::geometry::index::rstar<16>>> score_rtree_;
  RRTNode* score_parent_;
  double score_;
  std::shared_ptr<boost::geometry::index::rtree<value, boost::geometry::index::rstar<16>>> cost_rtree_;
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

  double getDistanceGain(std::shared_ptr<boost::geometry::index::rtree<value, boost::geometry::index::rstar<16>>> rtree,
                         double ltl_lambda, double min_distance, double max_distance, bool min_distance_active,
                         bool max_distance_active, double max_search_distance, double radius, int min_depth,
                         int max_depth)
  {
    if (!min_distance_active && !max_distance_active)
    {
      return 1;
    }

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

    std::pair<double, double> closest_distance =
        getDistanceToClosestOccupiedBounded(rtree, start, end, max_search_distance, radius, min_depth, max_depth);

    double distance_gain = 0;
    if (min_distance_active && max_distance_active)
    {
      distance_gain = std::min(closest_distance.first - min_distance, max_distance - closest_distance.first);
    }
    else if (min_distance_active)
    {
      distance_gain = closest_distance.first - min_distance;
    }
    else
    {
      distance_gain = max_distance - closest_distance.first;
    }

    return std::exp(-ltl_lambda * distance_gain);
  }

  double score(std::shared_ptr<boost::geometry::index::rtree<value, boost::geometry::index::rstar<16>>> rtree,
               double ltl_lambda, double min_distance, double max_distance, bool min_distance_active,
               bool max_distance_active, double max_search_distance, double radius, int min_depth, int max_depth,
               double lambda)
  {
    if (score_rtree_ && parent_ == score_parent_)
    {
      return score_;
    }

    score_rtree_ = rtree;
    score_parent_ = parent_;

    if (!this->parent_)
    {
      score_ = this->gain_;
      return score_;
    }

    double distance_gain = getDistanceGain(rtree, ltl_lambda, min_distance, max_distance, min_distance_active,
                                           max_distance_active, max_search_distance, radius, min_depth, max_depth);

    score_ = this->parent_->score(rtree, ltl_lambda, min_distance, max_distance, min_distance_active,
                                  max_distance_active, max_search_distance, radius, min_depth, max_depth, lambda) +
             this->gain_ * exp(-lambda * (this->distance(this->parent_) * std::fmax(distance_gain, 1)));
    return score_;
  }

  double cost(std::shared_ptr<boost::geometry::index::rtree<value, boost::geometry::index::rstar<16>>> rtree,
              double ltl_lambda, double min_distance, double max_distance, bool min_distance_active,
              bool max_distance_active, double max_search_distance, double radius, int min_depth, int max_depth)
  {
    if (cost_rtree_ && parent_ == cost_parent_)
    {
      return cost_;
    }

    cost_rtree_ = rtree;
    cost_parent_ = parent_;

    if (!this->parent_)
    {
      cost_ = 0;
      return cost_;
    }

    double distance_gain = getDistanceGain(rtree, ltl_lambda, min_distance, max_distance, min_distance_active,
                                           max_distance_active, max_search_distance, radius, min_depth, max_depth);

    cost_ = (this->distance(this->parent_) * std::fmax(distance_gain, 1)) +
            this->parent_->cost(rtree, ltl_lambda, min_distance, max_distance, min_distance_active, max_distance_active,
                                max_search_distance, radius, min_depth, max_depth);
    return cost_;
  }

  static boost::geometry::index::rtree<value, boost::geometry::index::rstar<16>>
  getRtree(std::shared_ptr<octomap::OcTree> ot, octomap::point3d min, octomap::point3d max)
  {
    boost::geometry::index::rtree<value, boost::geometry::index::rstar<16>> rtree;
    for (octomap::OcTree::leaf_bbx_iterator it = ot->begin_leafs_bbx(min, max), it_end = ot->end_leafs_bbx();
         it != it_end; ++it)
    {
      if (it->getLogOdds() > 0)
      {
        rtree.insert(point(it.getX(), it.getY(), it.getZ()));
      }
    }

    return rtree;
  }

  static std::pair<double, double> getDistanceToClosestOccupiedBounded(
      std::shared_ptr<boost::geometry::index::rtree<value, boost::geometry::index::rstar<16>>> rtree,
      Eigen::Vector3d start, Eigen::Vector3d end, double max_search_distance, double radius, int min_depth,
      int max_depth)
  {
    point bbx_min_(std::min(start[0] - radius, end[0] - max_search_distance),
                   std::min(start[1] - radius, end[1] - max_search_distance), std::min(start[2], end[2]) - radius);
    point bbx_max_(std::max(start[0] + radius, end[0] + max_search_distance),
                   std::max(start[1] + radius, end[1] + max_search_distance), std::max(start[2], end[2]) + radius);

    box query_box(bbx_min_, bbx_max_);
    std::vector<value> hits;
    rtree->query(boost::geometry::index::intersects(query_box), std::back_inserter(hits));

    int num_steps = 5;  // FIXME: Calculate

    std::vector<std::vector<double>> closest(omp_get_max_threads(), std::vector<double>(num_steps, 10000000));

#pragma omp parallel for
    for (int i = 0; i < hits.size(); ++i)
    {
      Eigen::Vector3d point(hits[i].get<0>(), hits[i].get<1>(), hits[i].get<2>());

      if (point[2] < std::min(start[2], end[2]) - 0.1 || point[2] > std::max(start[2], end[2]) + 0.1)
      {
        continue;
      }

      for (int j = 0; j < num_steps; ++j)
      {
        Eigen::Vector3d line_point;  // FIXME: Calculate

        double distance = (point - line_point).norm();

        if (distance > max_search_distance)
        {
          continue;
        }

        closest[omp_get_thread_num()][j] = std::min(closest[omp_get_thread_num()][j], distance);
      }
    }

    std::vector<double> final_closest(closest.size(), 10000000);
    for (size_t i = 0; i < closest.size(); ++i)
    {
      for (size_t j = 0; j < closest[i].size(); ++j)
      {
        final_closest[j] = std::min(final_closest[j], closest[i][j]);
      }
    }

    std::pair<std::vector<double>::iterator, std::vector<double>::iterator> minmax =
        std::minmax_element(final_closest.begin(), final_closest.end());
    return std::make_pair(*minmax.first, *minmax.second);
  }

  static double computeDistance(Eigen::Vector3d point, Eigen::Vector3d start, Eigen::Vector3d end)
  {
    if (start == end)
    {
      return (point - start).norm();
    }

    // Eigen::Vector3d d = (end - start) / (end - start).norm();
    // Eigen::Vector3d v = point - start;
    // double t = v.dot(d);
    // Eigen::Vector3d P = start + t * d;
    // return (point - P).norm();

    Eigen::Vector3d v = end - start;
    Eigen::Vector3d w = point - start;
    double c1 = w.dot(v);
    if (c1 <= 0)
    {
      return (point - start).norm();
    }
    double c2 = v.dot(v);
    if (c2 <= c1)
    {
      return (point - end).norm();
    }
    double b = c1 / c2;
    Eigen::Vector3d Pb = start + b * v;
    return (point - Pb).norm();
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
};  // namespace aeplanner
}  // namespace aeplanner

#endif
