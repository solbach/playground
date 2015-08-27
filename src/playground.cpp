#include "ros/ros.h"
#include "std_msgs/String.h"

/* Specifically for PCL usage */
#include "sensor_msgs/PointCloud2.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

ros::Publisher pub;

void cloudPub (const sensor_msgs::PointCloud2ConstPtr& msg)
{
    sensor_msgs::PointCloud2 output;
    output = *msg;

    ROS_INFO("I heard: [%i]", msg->width);
    pub.publish(output);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "playground");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/rtabmap/cloud_map", 1, cloudPub);
  pub = n.advertise<sensor_msgs::PointCloud2> ("playground/output", 1);

  ros::spin();

  return 0;
}
