#ifndef _AEPVIZ_H_
#define _AEPVIZ_H_

#include <string>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <eigen3/Eigen/Dense>
#include <tf/transform_datatypes.h>

#include <aeplanner/data_structures.h>

namespace aeplanner
{
visualization_msgs::MarkerArray createRRTMarkerArray(
    RRTNode* root, std::shared_ptr<octomap::OcTree> ot, Eigen::Vector4d current_state,
    double ltl_lambda, double min_distance, double max_distance, bool min_distance_active,
    bool max_distance_active, double max_search_distance, double radius, double lambda);
void recurse(RRTNode* node, visualization_msgs::MarkerArray* marker_array, int* id,
             std::shared_ptr<octomap::OcTree> ot, Eigen::Vector4d current_state,
             double ltl_lambda, double min_distance, double max_distance,
             bool min_distance_active, bool max_distance_active,
             double max_search_distance, double radius, double lambda);

visualization_msgs::Marker createNodeMarker(RRTNode* node, int id, std::string frame_id);
visualization_msgs::Marker createEdgeMarker(
    RRTNode* node, int id, std::string frame_id, std::shared_ptr<octomap::OcTree> ot,
    Eigen::Vector4d current_state, double ltl_lambda, double min_distance,
    double max_distance, bool min_distance_active, bool max_distance_active,
    double max_search_distance, double radius, double lambda);
}  // namespace aeplanner
#endif
