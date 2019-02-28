#include <aeplanner/aeplanner.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#include <aeplanner_msgs/LTLStats.h>
#include <numeric>

#include <queue>

#include <omp.h>

namespace aeplanner
{
AEPlanner::AEPlanner(const ros::NodeHandle& nh)
  : nh_(nh)
  , as_(nh_, "make_plan", boost::bind(&AEPlanner::execute, this, _1), false)
  , octomap_sub_(nh_.subscribe("octomap", 1, &AEPlanner::octomapCallback, this))
  , agent_pose_sub_(nh_.subscribe("agent_pose", 1, &AEPlanner::agentPoseCallback, this))
  , router_sub_(nh_.subscribe("/router", 10, &AEPlanner::routerCallback, this))
  , rrt_marker_pub_(nh_.advertise<visualization_msgs::MarkerArray>("rrtree", 1000))
  , gain_pub_(nh_.advertise<aeplanner_msgs::Node>("gain_node", 1000))
  , gp_query_client_(nh_.serviceClient<aeplanner_msgs::Query>("gp_query_server"))
  , reevaluate_server_(nh_.advertiseService("reevaluate", &AEPlanner::reevaluate, this))
  , best_node_client_(nh_.serviceClient<aeplanner_msgs::BestNode>("best_node_server"))
  , ot_(NULL)
  , stl_cs_(nh_)
  , rtree_(std::make_shared<value_rtree>())
  , stl_params_()
{
  // Set up dynamic reconfigure server
  stl_f_ = boost::bind(&AEPlanner::configCallback, this, _1, _2);
  stl_cs_.setCallback(stl_f_);

  params_ = readParams();
  max_sampling_radius_squared_ = pow(params_.max_sampling_radius, 2.0);

  as_.start();
}

void AEPlanner::execute(const aeplanner_msgs::aeplannerGoalConstPtr& goal)
{
  ROS_ERROR_STREAM("Execute start!");
  aeplanner_msgs::aeplannerResult result;

  // Check if aeplanner has recieved agent's pose yet
  if (!current_state_)
  {
    ROS_WARN("Agent's pose not yet received");
    ROS_WARN("Make sure it is being published and correctly mapped");
    as_.setSucceeded(result);
    return;
  }
  if (!ot_)
  {
    ROS_WARN("No octomap received");
    as_.setSucceeded(result);
    return;
  }

  std::shared_ptr<octomap::OcTree> ot = ot_;
  std::shared_ptr<Eigen::Vector4d> current_state = current_state_;

  octomap::point3d min(
      (*current_state)[0] - params_.max_sampling_radius - stl_params_.max_search_distance,
      (*current_state)[1] - params_.max_sampling_radius - stl_params_.max_search_distance,
      (*current_state)[2] - params_.max_sampling_radius -
          stl_params_.max_search_distance);

  octomap::point3d max(
      (*current_state)[0] + params_.max_sampling_radius + stl_params_.max_search_distance,
      (*current_state)[1] + params_.max_sampling_radius + stl_params_.max_search_distance,
      (*current_state)[2] + params_.max_sampling_radius +
          stl_params_.max_search_distance);

  std::shared_ptr<point_rtree> stl_rtree =
      std::make_shared<point_rtree>(getRtree(ot, min, max));

  if (!root_)
  {
    // Init RRT
    ROS_INFO("Init RRT");
    root_ = init(rtree_, current_state);
  }

  // Reevaluate potential information gain
  ROS_INFO("Reevaluate potential information gain");
  reevaluatePotentialInformationGainRecursive(root_);

  // Expand RRT
  ROS_INFO("Expand RRT");
  best_node_ = expandRRT(ot, rtree_, stl_rtree, current_state);

  // Publish RRT
  if (rrt_marker_pub_.getNumSubscribers() > 0)
  {
    ROS_INFO("Publish RRT");
    rrt_marker_pub_.publish(createRRTMarkerArray(root_, stl_rtree, current_state,
                                                 params_.bounding_radius, params_.lambda,
                                                 stl_params_));
  }

  // Get next goal
  ROS_INFO("Get next goal");
  std::shared_ptr<RRTNode> next_goal = getNextGoal(best_node_);

  // Update root to next node
  ROS_INFO("Update root");
  root_ = updateRoot(rtree_, stl_rtree, root_, next_goal, params_.extension_range,
                     params_.bounding_radius, params_.d_overshoot_);

  // Return next node
  ROS_INFO("Return next node");
  result.pose.pose = vecToPose(next_goal->state_);
  result.is_clear = true;
  as_.setSucceeded(result);

  // OLD

  // std::shared_ptr<value_rtree> rtree = std::make_shared<value_rtree>();  // rtree_;

  // ROS_WARN("Init");
  // std::shared_ptr<RRTNode> root = initialize(rtree, stl_rtree, current_state);
  // ROS_WARN("expandRRT");
  // ROS_WARN_STREAM(root->gain_ << " " << root->children_.size());
  // // if (root->gain_ > 0.75 or !root->children_.size() or
  // //     root->score(stl_rtree, ltl_lambda_, ltl_min_distance_, ltl_max_distance_,
  // //                 ltl_min_distance_active_, ltl_max_distance_active_,
  // //                 ltl_max_search_distance_, params_.bounding_radius, ltl_step_size_,
  // //                 ltl_routers_, ltl_routers_active_, params_.lambda) <
  // //                 params_.zero_gain)
  // // {
  // std::shared_ptr<RRTNode> best_node = expandRRT(ot, rtree, stl_rtree, current_state);
  // // }
  // // else
  // // {
  // //   best_node_ = root->children_[0];
  // // }

  // ROS_WARN("getCopyOfParent");
  // std::shared_ptr<RRTNode> best_branch_root = best_node->getCopyOfParentBranch();

  // ROS_WARN("createRRTMarker");
  // if (rrt_marker_pub_.getNumSubscribers() > 0)
  // {
  //   rrt_marker_pub_.publish(createRRTMarkerArray(root, stl_rtree, current_state,
  //                                                params_.bounding_radius,
  //                                                params_.lambda, stl_params_));
  // }
  // ROS_WARN("publishRecursive");
  // publishEvaluatedNodesRecursive(root);

  // ROS_WARN("extractPose");
  // result.pose.pose = vecToPose(best_branch_root->children_[0]->state_);
  // // if (best_node->score(stl_rtree, ltl_lambda_, ltl_min_distance_, ltl_max_distance_,
  // //                      ltl_min_distance_active_, ltl_max_distance_active_,
  // //                      ltl_max_search_distance_, params_.bounding_radius,
  // //                      ltl_step_size_, ltl_routers_, ltl_routers_active_,
  // //                      params_.lambda) > params_.zero_gain)
  // result.is_clear = true;
  // // else
  // // {
  // //   result.frontiers = getFrontiers();
  // //   result.is_clear = false;
  // // }
  // as_.setSucceeded(result);
}

std::shared_ptr<RRTNode> AEPlanner::init(std::shared_ptr<value_rtree> rtree,
                                         std::shared_ptr<Eigen::Vector4d> current_state)
{
  std::shared_ptr<RRTNode> root = std::make_shared<RRTNode>();
  root->state_ = *current_state;

  rtree->insert(
      std::make_pair(point(root->state_[0], root->state_[1], root->state_[2]), root));

  return root;
}

std::shared_ptr<RRTNode> AEPlanner::updateRoot(std::shared_ptr<value_rtree> rtree,
                                               std::shared_ptr<point_rtree> stl_rtree,
                                               std::shared_ptr<RRTNode> current_root,
                                               std::shared_ptr<RRTNode> new_root,
                                               double l, double r, double r_os)
{
  current_root->parent_ = new_root;
  // FIXME: Fix me, should always find it
  std::vector<std::shared_ptr<RRTNode>>::iterator it =
      std::find(current_root->children_.begin(), current_root->children_.end(), new_root);
  if (it != current_root->children_.end())
  {
    current_root->children_.erase(it);
  }

  new_root->parent_ = NULL;
  new_root->children_.push_back(current_root);

  std::vector<std::shared_ptr<RRTNode>> near =
      getNear(rtree, new_root, params_.extension_range);

  rewire(near, stl_rtree, current_root, new_root, params_.bounding_radius);

  return new_root;
}

point_rtree AEPlanner::getRtree(std::shared_ptr<octomap::OcTree> ot, octomap::point3d min,
                                octomap::point3d max)
{
  point_rtree rtree;
  for (octomap::OcTree::leaf_bbx_iterator it = ot->begin_leafs_bbx(min, max),
                                          it_end = ot->end_leafs_bbx();
       it != it_end; ++it)
  {
    if (it->getLogOdds() > 0)
    {
      rtree.insert(point(it.getX(), it.getY(), it.getZ()));
    }
  }

  return rtree;
}

std::shared_ptr<RRTNode> AEPlanner::initialize(std::shared_ptr<value_rtree> rtree,
                                               std::shared_ptr<point_rtree> stl_rtree,
                                               const Eigen::Vector4d& current_state)
{
  // std::shared_ptr<RRTNode> root = std::make_shared<RRTNode>();
  // root->state_[0] = current_state[0];
  // root->state_[1] = current_state[1];
  // root->state_[2] = current_state[2];

  // rtree->insert(
  //     std::make_pair(point(root->state_[0], root->state_[1], root->state_[2]), root));

  // if (root_)
  // {
  //   root->children_.push_back(root_);
  //   root_->parent_ = root;

  //   rewire(rtree, stl_rtree, root_, params_.extension_range, params_.bounding_radius,
  //          params_.d_overshoot_);
  // }

  // root_ = root;

  return root_;
}

void AEPlanner::initializeKDTreeWithPreviousBestBranch(std::shared_ptr<value_rtree> rtree,
                                                       std::shared_ptr<RRTNode> root)
{
  std::shared_ptr<RRTNode> current_node = root;
  do
  {
    rtree->insert(std::make_pair(
        point(current_node->state_[0], current_node->state_[1], current_node->state_[2]),
        current_node));
    if (current_node->children_.size())
      current_node = current_node->children_[0];
  } while (current_node->children_.size());
}

void AEPlanner::reevaluatePotentialInformationGainRecursive(std::shared_ptr<RRTNode> node)
{
  ROS_DEBUG_STREAM("Reevaluating!!");
  std::pair<double, double> ret = gainCubature(node->state_);
  node->state_[3] = ret.second;  // Assign yaw angle that maximizes g
  node->gain_ = ret.first;
  for (typename std::vector<std::shared_ptr<RRTNode>>::iterator node_it =
           node->children_.begin();
       node_it != node->children_.end(); ++node_it)
    reevaluatePotentialInformationGainRecursive(*node_it);
  ROS_DEBUG_STREAM("Reevaluating done!!");
}

std::shared_ptr<RRTNode> AEPlanner::expandRRT(
    std::shared_ptr<octomap::OcTree> ot, std::shared_ptr<value_rtree> rtree,
    std::shared_ptr<point_rtree> stl_rtree,
    std::shared_ptr<Eigen::Vector4d> current_state)
{
  // Expand an RRT tree and calculate information gain in every node
  for (int n = 0; n < params_.init_iterations and ros::ok(); ++n)
  {
    std::shared_ptr<RRTNode> new_node = std::make_shared<RRTNode>();
    std::shared_ptr<RRTNode> nearest;
    std::shared_ptr<RRTNode> parent;
    octomap::OcTreeNode* ot_result;

    // Sample new point around agent and check that
    // (1) it is within the boundaries
    // (2) it is in known space
    // (3) the path between the new node and it is parent does not contain any
    // obstacles

    while (true)
    {
      ROS_ERROR("Sample");
      Eigen::Vector4d offset = sampleNewPoint();
      new_node->state_ = *current_state + offset;

      ROS_ERROR("Nearest neighbour");
      nearest = nearestNeighbour(rtree, new_node);

      ROS_ERROR("Restrict distance");
      new_node->state_ = restrictDistance(nearest->state_, new_node->state_);

      ROS_ERROR("Find in octomap");
      ot_result = ot->search(octomap::point3d(new_node->state_[0], new_node->state_[1],
                                              new_node->state_[2]));

      ROS_ERROR("Check valid");
      if (ot_result && isInsideBoundaries(new_node->state_) &&
          collisionLine(stl_rtree, nearest->state_, new_node->state_,
                        params_.bounding_radius))
      {
        std::vector<std::shared_ptr<RRTNode>> near =
            getNear(rtree, new_node, params_.extension_range);

        ROS_ERROR("Choose parent");
        parent = chooseParent(near, stl_rtree, nearest, new_node);

        ROS_ERROR("Insert node");
        insertNode(rtree, parent, new_node);

        ROS_ERROR("Rewire");
        rewire(near, stl_rtree, parent, new_node, params_.bounding_radius);

        break;
      }
    }

    // Calculate potential information gain for new_node
    ROS_ERROR("Calculate gain");
    std::pair<double, double> ret = getGain(new_node);
    new_node->state_[3] = ret.second;  // Assign yaw angle that maximizes g
    new_node->gain_ = ret.first;

    // Update best node
    ROS_ERROR("Update best node");
    if (!best_node_ ||
        new_node->score(stl_rtree, params_.bounding_radius, params_.lambda, stl_params_) >
            best_node_->score(stl_rtree, params_.bounding_radius, params_.lambda,
                              stl_params_))
    {
      best_node_ = new_node;
    }
  }
  return best_node_;
}

Eigen::Vector4d AEPlanner::sampleNewPoint()
{
  // Samples one point uniformly over a sphere with a radius of
  // param_.max_sampling_radius
  Eigen::Vector4d point(0.0, 0.0, 0.0, 0.0);
  do
  {
    for (int i = 0; i < 3; i++)
      point[i] = params_.max_sampling_radius * 2.0 *
                 (((double)rand()) / ((double)RAND_MAX) - 0.5);
  } while (point.squaredNorm() > max_sampling_radius_squared_);

  return point;
}

std::shared_ptr<RRTNode> AEPlanner::nearestNeighbour(std::shared_ptr<value_rtree> rtree,
                                                     std::shared_ptr<RRTNode> new_node)
{
  std::vector<value> hits;
  rtree->query(
      boost::geometry::index::nearest(
          point(new_node->state_[0], new_node->state_[1], new_node->state_[2]), 1),
      std::back_inserter(hits));

  assert(hits.size() == 1);

  return hits[0].second;
}

std::vector<std::shared_ptr<RRTNode>> AEPlanner::getNear(
    std::shared_ptr<value_rtree> rtree, std::shared_ptr<RRTNode> new_node, double l)
{
  point bbx_min(new_node->state_[0] - l, new_node->state_[1] - l,
                new_node->state_[2] - l);
  point bbx_max(new_node->state_[0] + l, new_node->state_[1] + l,
                new_node->state_[2] + l);
  box query_box(bbx_min, bbx_max);

  std::vector<value> hits;
  rtree->query(boost::geometry::index::intersects(query_box), std::back_inserter(hits));

  std::vector<std::shared_ptr<RRTNode>> near;
  for (value item : hits)
  {
    near.push_back(item.second);
  }

  return near;
}

void AEPlanner::insertNode(std::shared_ptr<value_rtree> rtree,
                           std::shared_ptr<RRTNode> parent,
                           std::shared_ptr<RRTNode> new_node)
{
  new_node->parent_ = parent;
  parent->children_.push_back(new_node);

  rtree->insert(std::make_pair(
      point(new_node->state_[0], new_node->state_[1], new_node->state_[2]), new_node));
}

std::shared_ptr<RRTNode> AEPlanner::chooseParent(
    std::vector<std::shared_ptr<RRTNode>> near, std::shared_ptr<point_rtree> stl_rtree,
    std::shared_ptr<RRTNode> nearest_node, std::shared_ptr<RRTNode> new_node)
{
  std::shared_ptr<RRTNode> best_node = nearest_node;
  new_node->parent_ = nearest_node;  // Assign it so we can calculate the cost
  double best_cost = new_node->cost(stl_rtree, params_.bounding_radius, stl_params_);
  new_node->parent_ = NULL;  // Remove it since this is not the real parent yet

  for (std::shared_ptr<RRTNode> current_node : near)
  {
    if (collisionLine(stl_rtree, current_node->state_, new_node->state_,
                      params_.bounding_radius))
    {
      new_node->parent_ = current_node;  // Assign it so we can calculate the cost
      double current_cost =
          new_node->cost(stl_rtree, params_.bounding_radius, stl_params_);
      new_node->parent_ = NULL;  // Remove it since this is not the real parent yet

      if (current_cost < best_cost)
      {
        best_node = current_node;
        best_cost = current_cost;
      }
    }
  }

  return best_node;
}

void AEPlanner::rewire(std::vector<std::shared_ptr<RRTNode>> near,
                       std::shared_ptr<point_rtree> stl_rtree,
                       std::shared_ptr<RRTNode> nearest_node,
                       std::shared_ptr<RRTNode> new_node, double r)
{
  for (std::shared_ptr<RRTNode> current_node : near)
  {
    if (current_node == nearest_node || current_node == new_node)
    {
      continue;
    }

    if (!collisionLine(stl_rtree, new_node->state_, current_node->state_, r))
    {
      std::shared_ptr<RRTNode> current_parent = current_node->parent_;

      double current_cost =
          current_node->cost(stl_rtree, params_.bounding_radius, stl_params_);

      current_node->parent_ = new_node;  // Assign it so we can calculate the cost
      double new_cost =
          current_node->cost(stl_rtree, params_.bounding_radius, stl_params_);

      if (new_cost < current_cost)
      {
        std::vector<std::shared_ptr<RRTNode>>::iterator it =
            std::find(current_parent->children_.begin(), current_parent->children_.end(),
                      current_node);

        // FIXME: Should always be there
        if (it != current_parent->children_.end())
        {
          current_parent->children_.erase(it);
        }

        new_node->children_.push_back(current_node);
      }
      else
      {
        current_node->parent_ = current_parent;
      }
    }
  }
}

std::shared_ptr<RRTNode> AEPlanner::getNextGoal(std::shared_ptr<RRTNode> node)
{
  std::shared_ptr<RRTNode> current_node = node;

  while (current_node->parent_ && current_node->parent_->parent_)
  {
    current_node = current_node->parent_;
  }

  return current_node;
}

Eigen::Vector4d AEPlanner::restrictDistance(Eigen::Vector4d nearest,
                                            Eigen::Vector4d new_pos)
{
  // Check for collision
  Eigen::Vector3d origin(nearest[0], nearest[1], nearest[2]);
  Eigen::Vector3d direction(new_pos[0] - origin[0], new_pos[1] - origin[1],
                            new_pos[2] - origin[2]);
  // if (direction.norm() > params_.extension_range)
  if (direction.norm() > params_.extension_range)
    direction = params_.extension_range * direction.normalized();

  new_pos[0] = origin[0] + direction[0];
  new_pos[1] = origin[1] + direction[1];
  new_pos[2] = origin[2] + direction[2];

  return new_pos;
}

std::pair<double, double> AEPlanner::getGain(std::shared_ptr<RRTNode> node)
{
  aeplanner_msgs::Query srv;
  srv.request.point.x = node->state_[0];
  srv.request.point.y = node->state_[1];
  srv.request.point.z = node->state_[2];

  if (gp_query_client_.call(srv))
  {
    if (srv.response.sigma < params_.sigma_thresh)
    {
      double gain = srv.response.mu;
      double yaw = srv.response.yaw;
      return std::make_pair(gain, yaw);
    }
  }

  node->gain_explicitly_calculated_ = true;
  return gainCubature(node->state_);
}

bool AEPlanner::reevaluate(aeplanner_msgs::Reevaluate::Request& req,
                           aeplanner_msgs::Reevaluate::Response& res)
{
  ROS_DEBUG_STREAM("Reevaluate start!");
  for (std::vector<geometry_msgs::Point>::iterator it = req.point.begin();
       it != req.point.end(); ++it)
  {
    Eigen::Vector4d pos(it->x, it->y, it->z, 0);
    std::pair<double, double> gain_response = gainCubature(pos);
    res.gain.push_back(gain_response.first);
    res.yaw.push_back(gain_response.second);
  }
  ROS_DEBUG_STREAM("Reevaluate done!");

  return true;
}

std::vector<octomap::point3d> getChildren(octomap::point3d current,
                                          octomap::point3d origin, double resolution)
{
  octomap::point3d transformed = current - origin;

  // Source:
  // https://stackoverflow.com/questions/29557459/round-to-nearest-multiple-of-a-number
  // transformed.x() = int(((transformed.x() + resolution / 2.0) / resolution)) *
  // resolution; transformed.y() = int(((transformed.y() + resolution / 2.0) /
  // resolution)) * resolution; transformed.z() = int(((transformed.z() + resolution
  // / 2.0) / resolution)) * resolution;

  std::vector<octomap::point3d> children;

  std::vector<double> x_values;
  if (transformed.x() == 0)
  {
    x_values.push_back(current.x() - resolution);
    x_values.push_back(current.x());
    x_values.push_back(current.x() + resolution);
  }
  else if (transformed.y() == 0 && transformed.z() == 0)
  {
    double sign = (transformed.x() > 0) ? 1 : -1;

    x_values.push_back(current.x() + (sign * resolution));
  }
  else
  {
    double sign = (transformed.x() > 0) ? 1 : -1;

    x_values.push_back(current.x() + (sign * resolution));
    x_values.push_back(current.x());
  }

  std::vector<double> y_values;
  if (transformed.y() == 0)
  {
    y_values.push_back(current.y() - resolution);
    y_values.push_back(current.y());
    y_values.push_back(current.y() + resolution);
  }
  else if (transformed.y() == 0 && transformed.z() == 0)
  {
    double sign = (transformed.y() > 0) ? 1 : -1;

    y_values.push_back(current.y() + (sign * resolution));
  }
  else
  {
    double sign = (transformed.y() > 0) ? 1 : -1;

    y_values.push_back(current.y() + (sign * resolution));
    y_values.push_back(current.y());
  }

  std::vector<double> z_values;
  if (transformed.z() == 0)
  {
    z_values.push_back(current.z() - resolution);
    z_values.push_back(current.z());
    z_values.push_back(current.z() + resolution);
  }
  else if (transformed.x() == 0 && transformed.y() == 0)
  {
    double sign = (transformed.z() > 0) ? 1 : -1;

    z_values.push_back(current.z() + (sign * resolution));
  }
  else
  {
    double sign = (transformed.z() > 0) ? 1 : -1;

    z_values.push_back(current.z() + (sign * resolution));
    z_values.push_back(current.z());
  }

  for (double x : x_values)
  {
    for (double y : y_values)
    {
      for (double z : z_values)
      {
        if (x == 0 && y == 0 && z == 0)
        {
          // TODO: Maybe some rounding errors?!
          continue;
        }

        children.emplace_back(current.x() + x, current.y() + y, current.z() + z);
      }
    }
  }

  return children;
}

std::pair<double, double> AEPlanner::gainCubature(Eigen::Vector4d state)
{
  std::shared_ptr<octomap::OcTree> ot = ot_;

  // octomap::point3d origin(state[0], state[1], state[2]);

  // std::queue<octomap::point3d> open_set;
  // open_set.emplace(int(((origin.x() + ot->getResolution() / 2.0) /
  // ot->getResolution())) *
  //                      ot->getResolution(),
  //                  int(((origin.y() + ot->getResolution() / 2.0) /
  //                  ot->getResolution())) *
  //                      ot->getResolution(),
  //                  int(((origin.z() + ot->getResolution() / 2.0) /
  //                  ot->getResolution())) *
  //                      ot->getResolution());

  // std::set<std::tuple<double, double, double>> closed_set;
  // std::set<std::tuple<double, double, double>> unexplored_set;

  // std::vector<int> gain_per_yaw(360, 0);

  // double half_vfov = params_.vfov / 2.0;

  // octomap::point3d current;
  // while (!open_set.empty())
  // {
  //   current = open_set.front();
  //   open_set.pop();
  //   if (!closed_set.emplace(current.x(), current.y(), current.z()).second)
  //   {
  //     // Have already processed this
  //     continue;
  //   }

  //   octomap::OcTreeNode* result = ot->search(current);
  //   if (result)
  //   {
  //     if (result->getLogOdds() > 0)
  //     {
  //       // Already seen as Occupied space
  //       continue;
  //     }
  //   }
  //   else
  //   {
  //     if (!unexplored_set.emplace(current.x(), current.y(), current.z()).second)
  //     {
  //       // Already processed this
  //       continue;
  //     }
  //     else
  //     {
  //       // Is this correct?
  //       int yaw =
  //           std::round(std::atan2(origin.y() - current.y(), origin.x() - current.x())
  //           *
  //                      180.0 / M_PI);
  //       if (yaw < 0)
  //       {
  //         yaw += 360;
  //       }

  //       gain_per_yaw[yaw]++;
  //     }
  //   }

  //   // Get children
  //   std::vector<octomap::point3d> children =
  //       getChildren(current, origin, ot->getResolution());

  //   for (octomap::point3d child : children)
  //   {
  //     // Check if inside boundaries
  //     if (!isInsideBoundaries(child))
  //     {
  //       // Outside boundaries
  //       continue;
  //     }

  //     double distance = (child - origin).norm();

  //     // Check if in sensor fov
  //     double vertical_angle =
  //         std::fabs(std::asin((child - origin).z() / distance)) * 180.0 / M_PI;
  //     if (vertical_angle > half_vfov || distance > params_.r_max)
  //     {
  //       // Outside sensor fov or range
  //       continue;
  //     }

  //     open_set.push(child);
  //   }
  // }

  // // double gain = pow(ot->getResolution(), 3.0) * free_set.size();

  // int half_hfov = params_.hfov / 2;

  // int best_yaw = 0;
  // int best_yaw_score =
  //     std::accumulate(gain_per_yaw.begin(), gain_per_yaw.begin() + half_hfov, 0) +
  //     std::accumulate(gain_per_yaw.rbegin(), gain_per_yaw.rend() - half_hfov,
  //                     0);  // FIXME: Should the second one be -?
  // int previous_yaw_score = best_yaw_score;

  // for (int yaw = 1; yaw < 360; ++yaw)
  // {
  //   int current_yaw_score =
  //       previous_yaw_score -
  //       gain_per_yaw[(yaw - 1 - half_hfov + gain_per_yaw.size()) %
  //       gain_per_yaw.size()]
  //       + gain_per_yaw[(yaw + half_hfov) % gain_per_yaw.size()];

  //   previous_yaw_score = current_yaw_score;
  //   if (current_yaw_score > best_yaw_score)
  //   {
  //     best_yaw_score = current_yaw_score;
  //     best_yaw = yaw;
  //   }
  // }

  // double gain = pow(ot->getResolution(), 3.0) * best_yaw_score;

  // return std::make_pair(gain, best_yaw * M_PI / 180.0);

  double gain = 0.0;

  // This function computes the gain
  double fov_y = params_.hfov, fov_p = params_.vfov;

  double dr = params_.dr, dphi = params_.dphi, dtheta = params_.dtheta;
  double dphi_rad = M_PI * dphi / 180.0f, dtheta_rad = M_PI * dtheta / 180.0f;
  double r;
  int phi, theta;
  double phi_rad, theta_rad;

  std::map<int, double> gain_per_yaw;

  Eigen::Vector3d origin(state[0], state[1], state[2]);
  Eigen::Vector3d vec, dir;

  for (theta = -180; theta < 180; theta += dtheta)
  {
    theta_rad = M_PI * theta / 180.0f;
    for (phi = 90 - fov_p / 2; phi < 90 + fov_p / 2; phi += dphi)
    {
      phi_rad = M_PI * phi / 180.0f;

      double g = 0;
      for (r = params_.r_min; r < params_.r_max; r += dr)
      {
        vec[0] = state[0] + r * cos(theta_rad) * sin(phi_rad);
        vec[1] = state[1] + r * sin(theta_rad) * sin(phi_rad);
        vec[2] = state[2] + r * cos(phi_rad);
        dir = vec - origin;

        octomap::point3d query(vec[0], vec[1], vec[2]);
        octomap::OcTreeNode* result = ot->search(query);

        Eigen::Vector4d v(vec[0], vec[1], vec[2], 0);
        if (!isInsideBoundaries(v))
          break;
        if (result)
        {
          // Break if occupied so we don't count any information gain behind a wall.
          if (result->getLogOdds() > 0)
            break;
        }
        else
          g += (2 * r * r * dr + 1 / 6 * dr * dr * dr) * dtheta_rad * sin(phi_rad) *
               sin(dphi_rad / 2);
      }

      gain += g;
      gain_per_yaw[theta] += g;
    }
  }

  int best_yaw = 0;
  double best_yaw_score = 0;
  for (int yaw = -180; yaw < 180; yaw++)
  {
    double yaw_score = 0;
    for (int fov = -fov_y / 2; fov < fov_y / 2; fov++)
    {
      int theta = yaw + fov;
      if (theta < -180)
        theta += 360;
      if (theta > 180)
        theta -= 360;
      yaw_score += gain_per_yaw[theta];
    }

    if (best_yaw_score < yaw_score)
    {
      best_yaw_score = yaw_score;
      best_yaw = yaw;
    }
  }

  double r_max = params_.r_max;
  double h_max = params_.hfov / M_PI * 180;
  double v_max = params_.vfov / M_PI * 180;

  gain = best_yaw_score;  // / ((r_max*r_max*r_max/3) * h_max * (1-cos(v_max))) ;
  // ROS_ERROR_STREAM(gain);

  double yaw = M_PI * best_yaw / 180.f;

  state[3] = yaw;
  return std::make_pair(gain, yaw);
}

geometry_msgs::PoseArray AEPlanner::getFrontiers()
{
  geometry_msgs::PoseArray frontiers;

  aeplanner_msgs::BestNode srv;
  srv.request.threshold = 0.75;
  if (best_node_client_.call(srv))
  {
    for (int i = 0; i < srv.response.best_node.size(); ++i)
    {
      geometry_msgs::Pose frontier;
      frontier.position = srv.response.best_node[i];
      frontiers.poses.push_back(frontier);
    }
  }
  else
  {
  }

  return frontiers;
}

bool AEPlanner::isInsideBoundaries(Eigen::Vector4d point)
{
  return point[0] > params_.boundary_min[0] and point[0] < params_.boundary_max[0] and
         point[1] > params_.boundary_min[1] and point[1] < params_.boundary_max[1] and
         point[2] > params_.boundary_min[2] and point[2] < params_.boundary_max[2];
}

bool AEPlanner::isInsideBoundaries(Eigen::Vector3d point)
{
  return point[0] > params_.boundary_min[0] and point[0] < params_.boundary_max[0] and
         point[1] > params_.boundary_min[1] and point[1] < params_.boundary_max[1] and
         point[2] > params_.boundary_min[2] and point[2] < params_.boundary_max[2];
}

bool AEPlanner::isInsideBoundaries(octomap::point3d point)
{
  return point.x() > params_.boundary_min[0] and point.x() < params_.boundary_max[0] and
         point.y() > params_.boundary_min[1] and point.y() < params_.boundary_max[1] and
         point.z() > params_.boundary_min[2] and point.z() < params_.boundary_max[2];
}

bool AEPlanner::collisionLine(std::shared_ptr<point_rtree> stl_rtree, Eigen::Vector4d p1,
                              Eigen::Vector4d p2, double r)
{
  octomap::point3d start(p1[0], p1[1], p1[2]);
  octomap::point3d end(p2[0], p2[1], p2[2]);

  point bbx_min(std::min(p1[0], p2[0]) - r, std::min(p1[1], p2[1]) - r,
                std::min(p1[2], p2[2]) - r);
  point bbx_max(std::max(p1[0], p2[0]) + r, std::max(p1[1], p2[1]) + r,
                std::max(p1[2], p2[2]) + r);

  box query_box(bbx_min, bbx_max);
  std::vector<point> hits;
  stl_rtree->query(boost::geometry::index::intersects(query_box),
                   std::back_inserter(hits));

  double lsq = (end - start).norm_sq();
  double rsq = r * r;

  for (size_t i = 0; i < hits.size(); ++i)
  {
    octomap::point3d pt(hits[i].get<0>(), hits[i].get<1>(), hits[i].get<2>());

    if (CylTest_CapsFirst(start, end, lsq, rsq, pt) > 0 or (end - pt).norm() < r)
    {
      return true;
    }
  }

  return false;
}

void AEPlanner::octomapCallback(const octomap_msgs::Octomap& msg)
{
  ROS_DEBUG_STREAM("Freeing ot_");
  octomap::AbstractOcTree* aot = octomap_msgs::msgToMap(msg);
  octomap::OcTree* ot = (octomap::OcTree*)aot;
  ot_ = std::make_shared<octomap::OcTree>(*ot);

  delete ot;
  ROS_DEBUG_STREAM("Freeing ot_ done:");
}

void AEPlanner::publishEvaluatedNodesRecursive(std::shared_ptr<RRTNode> node)
{
  if (!node)
    return;
  for (typename std::vector<std::shared_ptr<RRTNode>>::iterator node_it =
           node->children_.begin();
       node_it != node->children_.end(); ++node_it)
  {
    if ((*node_it)->gain_explicitly_calculated_)
    {
      aeplanner_msgs::Node pig_node;
      pig_node.gain = (*node_it)->gain_;
      // ROS_ERROR_STREAM("GAIN: " << pig_node.gain);
      pig_node.pose.pose.position.x = (*node_it)->state_[0];
      pig_node.pose.pose.position.y = (*node_it)->state_[1];
      pig_node.pose.pose.position.z = (*node_it)->state_[2];
      tf2::Quaternion q;
      q.setRPY(0, 0, (*node_it)->state_[3]);
      pig_node.pose.pose.orientation.x = q.x();
      pig_node.pose.pose.orientation.y = q.y();
      pig_node.pose.pose.orientation.z = q.z();
      pig_node.pose.pose.orientation.w = q.w();
      gain_pub_.publish(pig_node);
    }

    publishEvaluatedNodesRecursive(*node_it);
  }
}

void AEPlanner::agentPoseCallback(const geometry_msgs::PoseStamped& msg)
{
  current_state_ = std::make_shared<Eigen::Vector4d>(
      msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
      tf2::getYaw(msg.pose.orientation));
}

geometry_msgs::Pose AEPlanner::vecToPose(Eigen::Vector4d state)
{
  tf::Vector3 origin(state[0], state[1], state[2]);
  double yaw = state[3];

  tf::Quaternion quat;
  quat.setEuler(0.0, 0.0, yaw);
  tf::Pose pose_tf(quat, origin);

  geometry_msgs::Pose pose;
  tf::poseTFToMsg(pose_tf, pose);

  return pose;
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
float AEPlanner::CylTest_CapsFirst(const octomap::point3d& pt1,
                                   const octomap::point3d& pt2, float lsq, float rsq,
                                   const octomap::point3d& pt)
{
  float dx, dy, dz;     // vector d  from line segment point 1 to point 2
  float pdx, pdy, pdz;  // vector pd from point 1 to test point
  float dot, dsq;

  dx = pt2.x() - pt1.x();  // translate so pt1 is origin.  Make vector from
  dy = pt2.y() - pt1.y();  // pt1 to pt2.  Need for this is easily eliminated
  dz = pt2.z() - pt1.z();

  pdx = pt.x() - pt1.x();  // vector from pt1 to test point.
  pdy = pt.y() - pt1.y();
  pdz = pt.z() - pt1.z();

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

struct
{
  bool operator()(const std::pair<octomap::point3d, double>& lhs,
                  const std::pair<octomap::point3d, double>& rhs) const
  {
    return lhs.second < rhs.second;
  }
} compareByDistance;

// void AEPlanner::createLTLSearchDistance()
// {
//   if (!ot_)
//   {
//     return;
//   }

//   ltl_search_distances_.clear();

//   double res = ot_->getResolution();

//   for (double x = -ltl_max_search_distance_; x <= 0; x += res)
//   {
//     for (double y = -ltl_max_search_distance_; y <= 0; y += res)
//     {
//       double distance = std::hypot(x, y);

//       if (distance <= ltl_max_search_distance_)
//       {
//         ltl_search_distances_.emplace_back(octomap::point3d(x, y, 0), distance);
//         ltl_search_distances_.emplace_back(octomap::point3d(x, -y, 0), distance);
//         ltl_search_distances_.emplace_back(octomap::point3d(-x, y, 0), distance);
//         ltl_search_distances_.emplace_back(octomap::point3d(-x, -y, 0), distance);
//       }
//     }
//   }

//   std::sort(ltl_search_distances_.begin(), ltl_search_distances_.end(),
//             compareByDistance);
// }

// double
// AEPlanner::getDistanceToClosestOccupiedBounded(std::shared_ptr<octomap::OcTree> ot,
//                                                       Eigen::Vector4d current_state)
// {
//   octomap::point3d state(current_state[0], current_state[1], current_state[2]);

//   for (std::pair<octomap::point3d, double> point : ltl_search_distances_)
//   {
//     octomap::OcTreeNode* node = ot->search(state + point.first);
//     if (node)
//     {
//       if (ot->isNodeOccupied(node))
//       {
//         return point.second;
//       }
//     }
//   }

//   return 10000000;
// }

void AEPlanner::configCallback(aeplanner::LTLConfig& config, uint32_t level)
{
  stl_params_.lambda = config.lambda;
  stl_params_.min_distance = config.min_distance;
  stl_params_.max_distance = config.max_distance;
  stl_params_.min_distance_active = config.min_distance_active;
  stl_params_.max_distance_active = config.max_distance_active;
  stl_params_.routers_active = config.routers_active;
  stl_params_.max_search_distance = config.max_search_distance;
  stl_params_.step_size = config.step_size;
}

void AEPlanner::routerCallback(const dd_gazebo_plugins::Router::ConstPtr& msg)
{
  stl_params_.routers[msg->id] = std::make_pair(msg->pose, msg->range);
}

}  // namespace aeplanner
