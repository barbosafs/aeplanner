#ifndef AEPLANNER_H
#define AEPLANNER_H

#include <ros/ros.h>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf/transform_listener.h>

#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>

#include <eigen3/Eigen/Dense>

#include <aeplanner/data_structures.h>
#include <aeplanner/param.h>
#include <aeplanner_msgs/Reevaluate.h>

#include <aeplanner/aeplanner_viz.h>
#include <visualization_msgs/MarkerArray.h>

#include <actionlib/server/simple_action_server.h>
#include <aeplanner_msgs/aeplannerAction.h>

#include <aeplanner_msgs/BestNode.h>
#include <aeplanner_msgs/Node.h>
#include <aeplanner_msgs/Query.h>

#include <aeplanner/LTLConfig.h>
#include <dynamic_reconfigure/server.h>
#include <nav_msgs/Path.h>

#include <dd_gazebo_plugins/Router.h>

#include <aeplanner/stl_params.h>

namespace aeplanner
{
typedef std::pair<point, std::shared_ptr<RRTNode>> value;
typedef boost::geometry::index::rtree<value, boost::geometry::index::rstar<16>>
    value_rtree;

class AEPlanner
{
private:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<aeplanner_msgs::aeplannerAction> as_;

  Params params_;

  // Current state of agent (x, y, z, yaw)
  std::shared_ptr<Eigen::Vector4d> current_state_;

  // The RRT
  std::shared_ptr<value_rtree> rtree_;

  // Keep track of the root of the RRT
  std::shared_ptr<RRTNode> root_;

  // Keep track of the best node
  std::shared_ptr<RRTNode> best_node_;

  // The map
  std::shared_ptr<octomap::OcTree> ot_;

  // Subscribers
  ros::Subscriber octomap_sub_;
  ros::Subscriber agent_pose_sub_;
  ros::Subscriber router_sub_;

  // Publishers
  ros::Publisher rrt_marker_pub_;
  ros::Publisher gain_pub_;

  // Services
  ros::ServiceClient best_node_client_;
  ros::ServiceClient gp_query_client_;
  ros::ServiceServer reevaluate_server_;

  // STL
  dynamic_reconfigure::Server<aeplanner::LTLConfig> stl_cs_;
  dynamic_reconfigure::Server<aeplanner::LTLConfig>::CallbackType stl_f_;
  STLParams stl_params_;

  double max_sampling_radius_squared_;

public:
  AEPlanner(const ros::NodeHandle& nh);

private:
  void execute(const aeplanner_msgs::aeplannerGoalConstPtr& goal);

  std::shared_ptr<RRTNode> init(std::shared_ptr<value_rtree> rtree,
                                std::shared_ptr<Eigen::Vector4d> current_state);

  std::shared_ptr<RRTNode> expandRRT(std::shared_ptr<octomap::OcTree> ot,
                                     std::shared_ptr<value_rtree> rtree,
                                     std::shared_ptr<point_rtree> stl_rtree,
                                     std::shared_ptr<Eigen::Vector4d> current_state);

  Eigen::Vector4d sampleNewPoint();

  std::shared_ptr<RRTNode> nearestNeighbour(std::shared_ptr<value_rtree> rtree,
                                            std::shared_ptr<RRTNode> new_node);

  std::vector<std::shared_ptr<RRTNode>> getNear(std::shared_ptr<value_rtree> rtree,
                                                std::shared_ptr<RRTNode> new_node,
                                                double l);

  void insertNode(std::shared_ptr<value_rtree> rtree, std::shared_ptr<RRTNode> parent,
                  std::shared_ptr<RRTNode> new_node);

  std::shared_ptr<RRTNode> chooseParent(std::vector<std::shared_ptr<RRTNode>> near,
                                        std::shared_ptr<point_rtree> stl_rtree,
                                        std::shared_ptr<RRTNode> nearest_node,
                                        std::shared_ptr<RRTNode> new_node);

  void rewire(std::vector<std::shared_ptr<RRTNode>> near,
              std::shared_ptr<point_rtree> stl_rtree,
              std::shared_ptr<RRTNode> nearest_node, std::shared_ptr<RRTNode> new_node,
              double r);

  std::shared_ptr<RRTNode> getNextGoal(std::shared_ptr<RRTNode> node);

  std::shared_ptr<RRTNode> updateRoot(std::shared_ptr<value_rtree> rtree,
                                      std::shared_ptr<point_rtree> stl_rtree,
                                      std::shared_ptr<RRTNode> current_root,
                                      std::shared_ptr<RRTNode> new_root, double l,
                                      double r, double r_os);

  void octomapCallback(const octomap_msgs::Octomap& msg);

  void agentPoseCallback(const geometry_msgs::PoseStamped& msg);

private:
  point_rtree getRtree(std::shared_ptr<octomap::OcTree> ot, octomap::point3d min,
                       octomap::point3d max);

  // Service server callback
  bool reevaluate(aeplanner_msgs::Reevaluate::Request& req,
                  aeplanner_msgs::Reevaluate::Response& res);

  // ---------------- Initialization ----------------
  std::shared_ptr<RRTNode> initialize(std::shared_ptr<value_rtree> rtree,
                                      std::shared_ptr<point_rtree> stl_rtree,
                                      const Eigen::Vector4d& current_state);
  void initializeKDTreeWithPreviousBestBranch(std::shared_ptr<value_rtree> rtree,
                                              std::shared_ptr<RRTNode> root);
  void reevaluatePotentialInformationGainRecursive(std::shared_ptr<RRTNode> node);

  // ---------------- Expand RRT Tree ----------------

  bool isInsideBoundaries(Eigen::Vector4d point);
  bool isInsideBoundaries(Eigen::Vector3d point);
  bool isInsideBoundaries(octomap::point3d point);
  bool collisionLine(std::shared_ptr<point_rtree> stl_rtree, Eigen::Vector4d p1,
                     Eigen::Vector4d p2, double r);
  Eigen::Vector4d restrictDistance(Eigen::Vector4d nearest, Eigen::Vector4d new_pos);

  std::pair<double, double> getGain(std::shared_ptr<RRTNode> node);
  std::pair<double, double> gainCubature(Eigen::Vector4d state);

  // ---------------- Helpers ----------------
  //
  void publishEvaluatedNodesRecursive(std::shared_ptr<RRTNode> node);

  geometry_msgs::Pose vecToPose(Eigen::Vector4d state);

  float CylTest_CapsFirst(const octomap::point3d& pt1, const octomap::point3d& pt2,
                          float lsq, float rsq, const octomap::point3d& pt);

  // ---------------- Frontier ----------------
  geometry_msgs::PoseArray getFrontiers();

  // LTL
  // void createLTLSearchDistance();
  // double getDistanceToClosestOccupiedBounded(std::shared_ptr<octomap::OcTree> ot,
  //                                            Eigen::Vector4d current_state);
  void configCallback(aeplanner::LTLConfig& config, uint32_t level);

  void routerCallback(const dd_gazebo_plugins::Router::ConstPtr& msg);
};

}  // namespace aeplanner

#endif
