#ifndef _AEPVIZ_H_
#define _AEPVIZ_H_

#include <string>

#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <eigen3/Eigen/Dense>

#include <aeplanner/data_structures.h>

namespace aeplanner
{
visualization_msgs::MarkerArray createRRTMarkerArray(
    std::shared_ptr<RRTNode> root, std::shared_ptr<point_rtree> rtree,
    Eigen::Vector4d current_state, double ltl_lambda, double min_distance,
    double max_distance, bool min_distance_active, bool max_distance_active,
    double max_search_distance, double radius, double step_size, double lambda);
void recurse(std::shared_ptr<RRTNode> node, visualization_msgs::MarkerArray* marker_array,
             int* id, std::shared_ptr<point_rtree> rtree, Eigen::Vector4d current_state,
             double ltl_lambda, double min_distance, double max_distance,
             bool min_distance_active, bool max_distance_active,
             double max_search_distance, double radius, double step_size, double lambda);

visualization_msgs::Marker createNodeMarker(std::shared_ptr<RRTNode> node, int id,
                                            std::string frame_id);
visualization_msgs::Marker createEdgeMarker(
    std::shared_ptr<RRTNode> node, int id, std::string frame_id,
    std::shared_ptr<point_rtree> rtree, Eigen::Vector4d current_state, double ltl_lambda,
    double min_distance, double max_distance, bool min_distance_active,
    bool max_distance_active, double max_search_distance, double radius, double step_size,
    double lambda);
}  // namespace aeplanner
#endif
