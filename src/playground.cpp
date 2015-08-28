/* Standard */
#include <iostream>
#include <vector>

/* ROS */
#include "ros/ros.h"
#include "std_msgs/String.h"

/* Specifically for PCL usage */
/* Segmentation */
#include "sensor_msgs/PointCloud2.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>

/* Identifying Ground */
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>

/*
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
*/

ros::Publisher pub;
pcl::visualization::CloudViewer viewer ("Segmentation Viewer");

void cloudSegmentation (const sensor_msgs::PointCloud2ConstPtr& msg)
{
    ROS_INFO("convert ros_msg to pcl");
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new
                    pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *cloud);

    ROS_INFO("Segmentation");
    pcl::search::Search<pcl::PointXYZ>::Ptr tree =
            boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> >
            (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud <pcl::Normal>::Ptr normals
            (new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod (tree);
    normal_estimator.setInputCloud (cloud);
    normal_estimator.setKSearch (50);
    normal_estimator.compute (*normals);

    pcl::IndicesPtr indices (new std::vector <int>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 1.0);
    pass.filter (*indices);

    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize (30);
    reg.setMaxClusterSize (300);
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (30);
    reg.setInputCloud (cloud);
    //reg.setIndices (indices);
    reg.setInputNormals (normals);
    reg.setSmoothnessThreshold (10.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold (5.0);

    std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters);

    std::cout << "Number of clusters is equal to " << clusters.size()
              << std::endl;

    std::cout << "Points in each cluster... " << clusters.size () << std::endl;

    for (int i = 0; i < clusters.size(); ++i) {
        std::cout << i << ". " << clusters[i].indices.size ()
                  << " points." << endl;
    }

    ROS_INFO("display");
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr segmented_cloud =
                    reg.getColoredCloud ();
    viewer.showCloud( segmented_cloud );

    pub.publish(msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "playground");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/rtabmap/cloud_map", 1, cloudSegmentation);
  pub = n.advertise<sensor_msgs::PointCloud2> ("playground/output", 1);

  ros::spin();

  return 0;
}
