#ifndef PIGAIN_GP_H
#define PIGAIN_GP_H

#include <ros/ros.h>

namespace pig
{
class PIG
{
private:
  ros::NodeHandle& nh_;

public:
  PIG(ros::NodeHandle& nh);
};
}  // namespace pig

#endif  // PIGAIN_GP_H