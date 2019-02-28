#include <aeplanner/aeplanner_viz.h>

namespace aeplanner
{
visualization_msgs::MarkerArray
createRRTMarkerArray(std::shared_ptr<RRTNode> root, std::shared_ptr<point_rtree> rtree,
                     std::shared_ptr<Eigen::Vector4d> current_state, double radius,
                     double lambda, const STLParams& stl_params)
{
  int id = 0;
  visualization_msgs::MarkerArray marker_array;
  recurse(root, &marker_array, &id, rtree, current_state, radius, lambda, stl_params);

  return marker_array;
}
void recurse(std::shared_ptr<RRTNode> node, visualization_msgs::MarkerArray* marker_array,
             int* id, std::shared_ptr<point_rtree> rtree,
             std::shared_ptr<Eigen::Vector4d> current_state, double radius, double lambda,
             const STLParams& stl_params)
{
  for (std::shared_ptr<RRTNode> child : node->children_)
  {
    // FIXME: Why is this if-statement here?
    if (child)
    {
      recurse(child, marker_array, id, rtree, current_state, radius, lambda, stl_params);
    }

    marker_array->markers.push_back(createEdgeMarker(
        child, (*id), "map", rtree, current_state, radius, lambda, stl_params));
    marker_array->markers.push_back(createNodeMarker(child, (*id)++, "map"));
  }
}

visualization_msgs::Marker createNodeMarker(std::shared_ptr<RRTNode> node, int id,
                                            std::string frame_id)
{
  visualization_msgs::Marker a;
  a.header.stamp = ros::Time::now();
  a.header.seq = id;
  a.header.frame_id = frame_id;
  a.id = id;
  a.ns = "vp_tree";
  a.type = visualization_msgs::Marker::ARROW;
  a.action = visualization_msgs::Marker::ADD;
  a.pose.position.x = node->state_[0];
  a.pose.position.y = node->state_[1];
  a.pose.position.z = node->state_[2];
  tf::Quaternion quat;
  quat.setEuler(0.0, 0.0, node->state_[3]);
  a.pose.orientation.x = quat.x();
  a.pose.orientation.y = quat.y();
  a.pose.orientation.z = quat.z();
  a.pose.orientation.w = quat.w();
  a.scale.x = std::max(node->gain_, 0.05);
  a.scale.y = 0.1;
  a.scale.z = 0.1;
  a.color.r = 167.0 / 255.0;
  a.color.g = 167.0 / 255.0;
  a.color.b = 0.0;
  a.color.a = 1.0;
  a.lifetime = ros::Duration(10.0);
  a.frame_locked = false;

  return a;
}

visualization_msgs::Marker createEdgeMarker(
    std::shared_ptr<RRTNode> node, int id, std::string frame_id,
    std::shared_ptr<point_rtree> rtree, std::shared_ptr<Eigen::Vector4d> current_state,
    double radius, double lambda, const STLParams& stl_params)
{
  visualization_msgs::Marker a;
  a.header.stamp = ros::Time::now();
  a.header.seq = id;
  a.header.frame_id = frame_id;
  a.id = id;
  a.ns = "vp_branches";
  a.type = visualization_msgs::Marker::ARROW;
  a.action = visualization_msgs::Marker::ADD;
  a.pose.position.x = node->parent_->state_[0];
  a.pose.position.y = node->parent_->state_[1];
  a.pose.position.z = node->parent_->state_[2];
  Eigen::Quaternion<double> q;
  Eigen::Vector3d init(1.0, 0.0, 0.0);
  Eigen::Vector3d dir(node->state_[0] - node->parent_->state_[0],
                      node->state_[1] - node->parent_->state_[1],
                      node->state_[2] - node->parent_->state_[2]);
  q.setFromTwoVectors(init, dir);
  q.normalize();
  a.pose.orientation.x = q.x();
  a.pose.orientation.y = q.y();
  a.pose.orientation.z = q.z();
  a.pose.orientation.w = q.w();
  a.scale.x = dir.norm();
  a.scale.y = 0.03;
  a.scale.z = 0.03;
  a.color.r = node->score(rtree, radius, lambda, stl_params) / 60.0;
  a.color.g = 0.0;
  a.color.b = 1.0;
  a.color.a = 1.0;
  a.lifetime = ros::Duration(10.0);
  a.frame_locked = false;

  return a;
}

}  // namespace aeplanner
