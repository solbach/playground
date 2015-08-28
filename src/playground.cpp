/* Standard */
#include <iostream>
#include <vector>

/* DEBUG */
#include <ctime>

/* ROS */
#include "ros/ros.h"
#include "std_msgs/String.h"

/* PCL */
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

ros::Publisher pub;
pcl::visualization::CloudViewer viewer ("Segmentation Viewer");

/* Set the mode for ground-elimination */
/*
 * 0 = Morphologically
 * 1 = Treshold
**/
uint8_t mode = 1;

pcl::PointCloud <pcl::PointXYZ>::Ptr cloudGroundFilterMorphological(
                                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new
                    pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointIndicesPtr ground (new pcl::PointIndices);

    /* create filter object and extract ground */

    pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
    pmf.setInputCloud(cloud);
    pmf.setMaxWindowSize(20);
    pmf.setSlope(1.0f);
    pmf.setInitialDistance(0.5f);
    pmf.setMaxDistance(3.0f);
    pmf.extract(ground->indices);
    /* extract ground from cloud and save to cloud_filtered */
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(ground);
    extract.filter(*cloud);
    extract.setNegative(true);
    extract.filter(*cloud_filtered);

    return cloud_filtered;
}

pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloudSegmentation(
                 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered)
{
    pcl::search::Search<pcl::PointXYZ>::Ptr tree =
            boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> >
            (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud <pcl::Normal>::Ptr normals
            (new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod (tree);
    normal_estimator.setInputCloud (cloud_filtered);
    normal_estimator.setKSearch (50);
    normal_estimator.compute (*normals);

    pcl::IndicesPtr indices (new std::vector <int>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud_filtered);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 1.0);
    pass.filter (*indices);

    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize (30);
    reg.setMaxClusterSize (300);
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (30);
    reg.setInputCloud (cloud_filtered);
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
                    reg.getColoredCloud();

    return segmented_cloud;
}

pcl::PointCloud <pcl::PointXYZ>::Ptr cloudGroundFilteringTreshold(
                 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    float treshold = 0.5f;

    /* Iterate over cloud */
    for (pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud->begin();
         it != cloud->end(); it++)
    {
        std::cout << "IN!" << std::endl;
    }

    std::cout << "OUT!" << std::endl;
}

void cloudSubscriber (const sensor_msgs::PointCloud2ConstPtr& msg)
{
    const clock_t t0=clock();
    /* base cloud */
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new
                    pcl::PointCloud<pcl::PointXYZ>());
    /* convert ros message to base cloud */
    pcl::fromROSMsg(*msg, *cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new
                    pcl::PointCloud<pcl::PointXYZ>);

    if (mode == 0)
    {
//        ROS_INFO("Ground Filtering Morphologicaly");
        /* filter cloud */
        cloud_filtered = cloudGroundFilterMorphological(cloud);
    }
    else if(mode == 1)
    {
//        ROS_INFO("Ground Filtering Treshold");
        /* filter cloud */
        cloud_filtered = cloudGroundFilterMorphological(cloud);
    }

//    ROS_INFO("Segmentation");
    /* segment cloud */
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr segmented_cloud =
                                            cloudSegmentation(cloud_filtered);
//    std::cout << "TIME / SIZE ::: " << float( clock () - t0 ) /  CLOCKS_PER_SEC
//              << ", " << msg->width << std::endl;

    viewer.showCloud( segmented_cloud );

    pub.publish(msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "playground");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/rtabmap/cloud_map", 1, cloudSubscriber);
  pub = n.advertise<sensor_msgs::PointCloud2> ("playground/output", 1);

  ros::spin();

  return 0;
}
