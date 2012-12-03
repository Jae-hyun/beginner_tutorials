#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// PCL statistical outlier removal include
#include <pcl/filters/statistical_outlier_removal.h>

       
ros::Publisher pub;
void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  // ... do data processing
  sensor_msgs::PointCloud2 cloudFiltered;
  std::cerr << "Before Voxel filtering: " << cloud->width * cloud->height  << " data points.\n";

  //Perform the actual filtering
  // Create the filtering object
  pcl::StatisticalOutlierRemoval<sensor_msgs::PointCloud2> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter (cloudFiltered);

  // Publish the data
  pub.publish (cloudFiltered);

  std::cerr << "After Voxel filtering: " << cloudFiltered.width * cloudFiltered.height  << " data points.\n";



}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pcl_statisticalFilter");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("camera/depth_registered/points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("cloudStatisticalFiltered", 1);

  // Spin
  ros::spin ();
}
/*
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

  int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;

  // Fill in the cloud data
  cloud.width  = 15;
  cloud.height = 1;
  cloud.points.resize (cloud.width * cloud.height);

  // Generate the data
  for (size_t i = 0; i < cloud.points.size (); ++i)
  {
    cloud.points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud.points[i].z = 1.0;
  }

  // Set a few outliers
  cloud.points[0].z = 2.0;
  cloud.points[3].z = -2.0;
  cloud.points[6].z = 4.0;

  std::cerr << "Point cloud data: " << cloud.points.size () << " points" << std::endl;
  for (size_t i = 0; i < cloud.points.size (); ++i)
    std::cerr << "    " << cloud.points[i].x << " " 
      << cloud.points[i].y << " " 
      << cloud.points[i].z << std::endl;

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud (cloud.makeShared ());
  seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    return (-1);
  }

  std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
    << coefficients->values[1] << " "
    << coefficients->values[2] << " " 
    << coefficients->values[3] << std::endl;

  std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
  for (size_t i = 0; i < inliers->indices.size (); ++i)
    std::cerr << inliers->indices[i] << "    " << cloud.points[inliers->indices[i]].x << " "
      << cloud.points[inliers->indices[i]].y << " "
      << cloud.points[inliers->indices[i]].z << std::endl;

  return (0);
}
*/
/*
// Create the filtering object
pcl::ExtractIndices<pcl::PointXYZ> extract;

int i = 0, nr_points = (int) cloud_filtered->points.size ();
// While 30% of the original cloud is still there
while (cloud_filtered->points.size () > 0.3 * nr_points)
{
  // Segment the largest planar component from the remaining cloud
  seg.setInputCloud (cloud_filtered);
  seg.segment (*inliers, *coefficients);
  if (inliers->indices.size () == 0)
  {
    std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    break;
  }

  // Extract the inliers
  extract.setInputCloud (cloud_filtered);
  extract.setIndices (inliers);
  extract.setNegative (false);
  extract.filter (*cloud_p);
  std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

  std::stringstream ss;
  ss << "table_scene_lms400_plane_" << i << ".pcd";
  writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);

  // Create the filtering object
  extract.setNegative (true);
  extract.filter (*cloud_f);
  cloud_filtered.swap (cloud_f);
  i++;
}
*/
