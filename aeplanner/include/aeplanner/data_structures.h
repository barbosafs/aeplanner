#ifndef DATA_STRUCTURES_H
#define DATA_STRUCTURES_H

#include <ros/ros.h>
#include <octomap/OcTree.h>

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
  double score_;
  std::shared_ptr<octomap::OcTree> cost_ot_;
  double cost_;

  RRTNode() : parent_(NULL), gain_(0.0), gain_explicitly_calculated_(false)
  {
  }
  ~RRTNode()
  {
    for (typename std::vector<RRTNode*>::iterator node_it = children_.begin();
         node_it != children_.end(); ++node_it)
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

  double getDistanceGain(
      std::shared_ptr<octomap::OcTree> ot, double ltl_lambda, double min_distance,
      double max_distance, bool min_distance_active, bool max_distance_active,
      std::vector<std::pair<octomap::point3d, double>> search_distances)
  {
    double closest_distance = 0;
    if (min_distance_active || max_distance_active)
    {
      closest_distance = getDistanceToClosestOccupiedBounded(ot, search_distances);
    }

    double distance_gain = 0;
    if (min_distance_active && max_distance_active)
    {
      distance_gain =
          std::min(closest_distance - min_distance, max_distance - closest_distance);
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

  double score(std::shared_ptr<octomap::OcTree> ot, double ltl_lambda,
               double min_distance, double max_distance, bool min_distance_active,
               bool max_distance_active,
               std::vector<std::pair<octomap::point3d, double>> search_distances,
               double lambda)
  {
    if (score_ot_ && ot == score_ot_)
    {
      return score_;
    }

    score_ot_ = ot;

    if (!this->parent_)
    {
      score_ = this->gain_;
      return score_;
    }

    double distance_gain =
        getDistanceGain(ot, ltl_lambda, min_distance, max_distance, min_distance_active,
                        max_distance_active, search_distances);

    score_ =
        this->parent_->score(ot, ltl_lambda, min_distance, max_distance,
                             min_distance_active, max_distance_active, search_distances,
                             lambda) +
        this->gain_ *
            exp(-lambda * (this->distance(this->parent_) * std::fmax(distance_gain, 1)));
    return score_;
  }

  double cost(std::shared_ptr<octomap::OcTree> ot, double ltl_lambda, double min_distance,
              double max_distance, bool min_distance_active, bool max_distance_active,
              std::vector<std::pair<octomap::point3d, double>> search_distances)
  {
    if (cost_ot_ && ot == cost_ot_)
    {
      return cost_;
    }

    cost_ot_ = ot;

    if (!this->parent_)
    {
      cost_ = 0;
      return cost_;
    }

    double distance_gain =
        getDistanceGain(ot, ltl_lambda, min_distance, max_distance, min_distance_active,
                        max_distance_active, search_distances);

    cost_ =
        (this->distance(this->parent_) * std::fmax(distance_gain, 1)) +
        this->parent_->cost(ot, ltl_lambda, min_distance, max_distance,
                            min_distance_active, max_distance_active, search_distances);
    return cost_;
  }

  double getDistanceToClosestOccupiedBounded(
      std::shared_ptr<octomap::OcTree> ot,
      std::vector<std::pair<octomap::point3d, double>> search_distances)
  {
    octomap::point3d state(state_[0], state_[1], state_[2]);

    for (std::pair<octomap::point3d, double> point : search_distances)
    {
      octomap::OcTreeNode* node = ot->search(state + point.first);
      if (node)
      {
        if (ot->isNodeOccupied(node))
        {
          return point.second;
        }
      }
    }

    return 1000000;
  }

  double distance(RRTNode* other)
  {
    Eigen::Vector3d p3(this->state_[0], this->state_[1], this->state_[2]);
    Eigen::Vector3d q3(other->state_[0], other->state_[1], other->state_[2]);
    return (p3 - q3).norm();
  }
};
}  // namespace aeplanner

#endif
