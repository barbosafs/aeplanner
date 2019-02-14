#include <fstream>

#include <ros/package.h>
#include <ros/ros.h>
#include <tf/tf.h>

#include <aeplanner_msgs/Coverage.h>
#include <std_srvs/Empty.h>

#include <actionlib/client/simple_action_client.h>
#include <aeplanner_msgs/FlyToAction.h>
#include <aeplanner_msgs/Node.h>
#include <aeplanner_msgs/aeplannerAction.h>
#include <aeplanner_msgs/rrtAction.h>

#include <geometry_msgs/PoseStamped.h>

#include <nav_msgs/Path.h>
#include <tf2/utils.h>

#include <eigen3/Eigen/Eigen>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "exploration");
  ros::NodeHandle nh;
  ROS_INFO("Started exploration");

  // Open logfile;
  std::string path = ros::package::getPath("rpl_exploration");
  std::ofstream logfile, pathfile;
  logfile.open(path + "/data/logfile.csv");
  pathfile.open(path + "/data/path.csv");

  ros::Publisher pub(nh.advertise<geometry_msgs::PoseStamped>("/mavros/"
                                                              "setpoint_position/"
                                                              "local",
                                                              1000));
  ros::ServiceClient coverage_srv = nh.serviceClient<aeplanner_msgs::Coverage>("/ge"
                                                                               "t_"
                                                                               "cov"
                                                                               "era"
                                                                               "g"
                                                                               "e");

  // wait for fly_to server to start
  // ROS_INFO("Waiting for fly_to action server");
  actionlib::SimpleActionClient<aeplanner_msgs::FlyToAction> ac("fly_to", true);
  // ac.waitForServer(); //will wait for infinite time
  // ROS_INFO("Fly to ction server started!");

  // wait for aep server to start
  ROS_INFO("Waiting for aeplanner action server");
  actionlib::SimpleActionClient<aeplanner_msgs::aeplannerAction> aep_ac("make_plan", true);
  aep_ac.waitForServer();  // will wait for infinite time
  ROS_INFO("aeplanner action server started!");

  // wait for fly_to server to start
  ROS_INFO("Waiting for rrt action server");
  actionlib::SimpleActionClient<aeplanner_msgs::rrtAction> rrt_ac("rrt", true);
  // rrt_ac.waitForServer(); //will wait for infinite time
  ROS_INFO("rrt Action server started!");

  // Get current pose
  geometry_msgs::PoseStamped::ConstPtr init_pose = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/mavros/"
                                                                                                          "local_"
                                                                                                          "position/"
                                                                                                          "pose");
  double init_yaw = tf2::getYaw(init_pose->pose.orientation);
  // Up 2 meters and then forward one meter
  std::vector<Eigen::Vector4d> initial_positions;
  initial_positions.emplace_back(init_pose->pose.position.x, init_pose->pose.position.y, 0.0, init_yaw);
  initial_positions.emplace_back(init_pose->pose.position.x, init_pose->pose.position.y, 0.5, init_yaw);
  double move_forward_distance = 0.0;
  double yaw_radians = M_PI;
  initial_positions.emplace_back(init_pose->pose.position.x + move_forward_distance * std::cos(init_yaw),
                                 init_pose->pose.position.y + move_forward_distance * std::sin(init_yaw), 0.5,
                                 init_yaw + (yaw_radians / 2));
  initial_positions.emplace_back(init_pose->pose.position.x + move_forward_distance * std::cos(init_yaw),
                                 init_pose->pose.position.y + move_forward_distance * std::sin(init_yaw), 0.5,
                                 init_yaw + yaw_radians);
  initial_positions.emplace_back(init_pose->pose.position.x + move_forward_distance * std::cos(init_yaw),
                                 init_pose->pose.position.y + move_forward_distance * std::sin(init_yaw), 0.5,
                                 init_yaw - (yaw_radians / 2));
  initial_positions.emplace_back(init_pose->pose.position.x + move_forward_distance * std::cos(init_yaw),
                                 init_pose->pose.position.y + move_forward_distance * std::sin(init_yaw), 0.5,
                                 init_yaw);

  // This is the initialization motion, necessary that the known free space
  // allows the planning of initial paths.
  ROS_INFO("Starting the planner: Performing initialization motion");
  geometry_msgs::PoseStamped last_pose;

  for (int i = 0; i < initial_positions.size(); ++i)
  {
    aeplanner_msgs::FlyToGoal goal;
    goal.pose.pose.position.x = initial_positions[i][0];
    goal.pose.pose.position.y = initial_positions[i][1];
    goal.pose.pose.position.z = initial_positions[i][2];
    goal.pose.pose.orientation = tf::createQuaternionMsgFromYaw(initial_positions[i][3]);
    goal.distance_converged = 0.1;
    goal.yaw_converged = 0.1 * M_PI;
    last_pose.pose = goal.pose.pose;

    ROS_INFO("Sending initial goal %d out of %d...", (i + 1), (int)initial_positions.size());
    ac.sendGoal(goal);

    ac.waitForResult(ros::Duration(0));
  }

  // Start planning: The planner is called and the computed path sent to the
  // controller.
  int iteration = 0;
  int actions_taken = 1;

  ros::Time start = ros::Time::now();
  while (ros::ok())
  {
    ROS_INFO_STREAM("Planning iteration " << iteration);
    aeplanner_msgs::aeplannerGoal aep_goal;
    aep_goal.header.stamp = ros::Time::now();
    aep_goal.header.seq = iteration;
    aep_goal.header.frame_id = "map";
    aep_goal.actions_taken = actions_taken;
    aep_ac.sendGoal(aep_goal);

    while (!aep_ac.waitForResult(ros::Duration(0.05)))
    {
      pub.publish(last_pose);
    }

    ros::Duration fly_time;
    if (aep_ac.getResult()->is_clear)
    {
      actions_taken = 0;

      ros::Time s = ros::Time::now();
      geometry_msgs::PoseStamped goal_pose = aep_ac.getResult()->pose;
      // Write path to file
      pathfile << goal_pose.pose.position.x << ", " << goal_pose.pose.position.y << ", " << goal_pose.pose.position.z
               << ", n" << std::endl;

      last_pose.pose = goal_pose.pose;
      aeplanner_msgs::FlyToGoal goal;
      goal.pose = goal_pose;
      goal.distance_converged = 0.8;
      goal.yaw_converged = 0.8 * M_PI;
      ac.sendGoal(goal);

      ac.waitForResult(ros::Duration(0));

      fly_time = ros::Time::now() - s;
    }
    else
    {
      aeplanner_msgs::rrtGoal rrt_goal;
      rrt_goal.start.header.stamp = ros::Time::now();
      rrt_goal.start.header.frame_id = "map";
      rrt_goal.start.pose = last_pose.pose;
      if (!aep_ac.getResult()->frontiers.poses.size())
      {
        ROS_WARN("Exploration complete!");
        break;
      }
      for (auto it = aep_ac.getResult()->frontiers.poses.begin(); it != aep_ac.getResult()->frontiers.poses.end(); ++it)
      {
        rrt_goal.goal_poses.poses.push_back(*it);
      }

      rrt_ac.sendGoal(rrt_goal);
      while (!rrt_ac.waitForResult(ros::Duration(0.05)))
      {
        pub.publish(last_pose);
      }
      nav_msgs::Path path = rrt_ac.getResult()->path;

      ros::Time s = ros::Time::now();
      for (int i = path.poses.size() - 1; i >= 0; --i)
      {
        geometry_msgs::Pose goal_pose = path.poses[i].pose;
        // Write path to file
        pathfile << goal_pose.position.x << ", " << goal_pose.position.y << ", " << goal_pose.position.z << ", f"
                 << std::endl;

        last_pose.pose = goal_pose;
        aeplanner_msgs::FlyToGoal goal;
        goal.pose.pose = goal_pose;
        goal.distance_converged = 0.8;
        goal.yaw_converged = 0.8 * M_PI;
        ac.sendGoal(goal);

        ac.waitForResult(ros::Duration(0));
      }
      actions_taken = -1;
      fly_time = ros::Time::now() - s;
    }

    ros::Duration elapsed = ros::Time::now() - start;

    aeplanner_msgs::Coverage srv;
    if (!coverage_srv.call(srv))
    {
      ROS_ERROR("Failed to call coverage service");
    }

    ROS_INFO_STREAM("Iteration: " << iteration << "  "
                                  << "Time: " << elapsed << "  "
                                  << "Sampling: " << aep_ac.getResult()->sampling_time.data << "  "
                                  << "Planning: " << aep_ac.getResult()->planning_time.data << "  "
                                  << "Collision check: " << aep_ac.getResult()->collision_check_time.data << "  "
                                  << "Flying: " << fly_time << " "
                                  << "Tree size: " << aep_ac.getResult()->tree_size << " "
                                  << "Coverage: " << srv.response.coverage << " "
                                  << "F:     " << srv.response.free << " "
                                  << "O: " << srv.response.occupied << " "
                                  << "U: " << srv.response.unmapped);

    logfile << iteration << ", " << elapsed << ", " << aep_ac.getResult()->sampling_time.data << ", "
            << aep_ac.getResult()->planning_time.data << ", " << aep_ac.getResult()->collision_check_time.data << ", "
            << fly_time << ", " << srv.response.coverage << ", " << srv.response.free << ", " << srv.response.occupied
            << ", " << srv.response.unmapped << ", " << aep_ac.getResult()->tree_size << std::endl;

    iteration++;
  }

  pathfile.close();
  logfile.close();
}
