#include <ros/ros.h>
#include <ros/package.h>
#include <aeplanner_msgs/LTLStats.h>
#include <aeplanner_msgs/Volume.h>
#include <geometry_msgs/PoseStamped.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <tf2/utils.h>

#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>

std::ofstream stats_file;
std::ofstream pose_file;
std::ofstream map_file;

double total_volume;
octomap::point3d bbx_min;
octomap::point3d bbx_max;

ros::Publisher volume_pub;
double volume_scaler;

void LTLStatsCallback(const aeplanner_msgs::LTLStats::ConstPtr& stats)
{
  if (stats_file.is_open())
  {
    stats_file << stats->header.stamp << ", ";
    stats_file << stats->ltl_min_distance << ", ";
    stats_file << stats->ltl_max_distance << ", ";
    stats_file << stats->current_closest_distance << ", ";
    stats_file << stats->mean_closest_distance << "\n";
    stats_file.flush();
  }
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
  if (pose_file.is_open())
  {
    pose_file << pose->header.stamp << ", ";
    pose_file << pose->pose.position.x << ", ";
    pose_file << pose->pose.position.y << ", ";
    pose_file << pose->pose.position.z << ", ";
    pose_file << tf2::getYaw(pose->pose.orientation) << "\n";
    pose_file.flush();
  }
}

void mapCallback(const octomap_msgs::Octomap::ConstPtr& map)
{
  if (map_file.is_open())
  {
    octomap::OcTree* ot = (octomap::OcTree*)octomap_msgs::msgToMap(*map);

    double res = ot->getResolution();
    double cell_size = std::pow(res, 3.0);

    double volume_unmapped = 0;
    double volume_occupied = 0;
    double volume_free = 0;
    for (double x = bbx_min.x(); x < bbx_max.x() - res / 2; x += res)
    {
      for (double y = bbx_min.y(); y < bbx_max.y() - res / 2; y += res)
      {
        for (double z = bbx_min.z(); z < bbx_max.z() - res / 2; z += res)
        {
          octomap::OcTreeNode* result = ot->search(x + res / 2, y + res / 2, z + res / 2);
          if (!result)
          {
            volume_unmapped += cell_size;
          }
          else if (result->getLogOdds() > 0)
          {
            volume_occupied += cell_size;
          }
          else
          {
            volume_free += cell_size;
          }
        }
      }
    }

    delete ot;

    map_file << map->header.stamp << ", ";
    map_file << total_volume << ", ";
    map_file << (volume_free + volume_occupied) << "\n";
    map_file.flush();

    aeplanner_msgs::Volume msg;
    msg.header.stamp = map->header.stamp;
    msg.current_volume = (volume_free + volume_occupied) * volume_scaler / total_volume;
    volume_pub.publish(msg);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "stl_evaluation");

  std::string package_path = ros::package::getPath("stl_evaluation");

  boost::filesystem::create_directory(package_path + "/data/");

  int postfix = 0;
  while (boost::filesystem::exists(package_path + "/data/stats_" +
                                   std::to_string(postfix) + ".txt"))
  {
    ++postfix;
  }

  stats_file.open(package_path + "/data/stats_" + std::to_string(postfix) + ".txt");
  stats_file << "Stamp, Min, Max, Current, Mean\n";

  pose_file.open(package_path + "/data/pose_" + std::to_string(postfix) + ".txt");
  pose_file << "Stamp, X, Y, Z, Yaw\n";

  map_file.open(package_path + "/data/map_" + std::to_string(postfix) + ".txt");
  map_file << "Stamp, Max volume, Current volume\n";

  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  bbx_min.x() = nh_priv.param("bbx_min_x", -50);
  bbx_min.y() = nh_priv.param("bbx_min_y", -50);
  bbx_min.z() = nh_priv.param("bbx_min_z", 0);

  bbx_max.x() = nh_priv.param("bbx_max_x", 50);
  bbx_max.y() = nh_priv.param("bbx_max_y", 50);
  bbx_max.z() = nh_priv.param("bbx_max_z", 2.8);

  volume_scaler = nh_priv.param("volume_scaler", 1.0);

  octomap::point3d temp = bbx_max - bbx_min;
  total_volume = temp.x() * temp.y() * temp.z();

  ros::Subscriber ltl_sub = nh.subscribe("/aeplanner/ltl_stats", 1000, &LTLStatsCallback);
  ros::Subscriber pose_sub =
      nh.subscribe("/mavros/local_position/pose", 1000, &poseCallback);
  ros::Subscriber map_sub = nh.subscribe("/aeplanner/octomap_full", 1000, &mapCallback);

  volume_pub = nh.advertise<aeplanner_msgs::Volume>("/stl_evaluation/volume", 1000);

  ros::spin();

  stats_file.close();
  pose_file.close();
  map_file.close();

  return 0;
}