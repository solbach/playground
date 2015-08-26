#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"

void chatterCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  ROS_INFO("I heard: [%i]", msg->row_step);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/rtabmap/cloud_map", 1000, chatterCallback);
  ros::spin();

  return 0;
}
