#include <ros/ros.h>
#include <pigain/pig.h>

namespace pig
{
PIG::PIG(ros::NodeHandle& nh) : nh_(nh)
{
}
}  // namespace pig