#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// PCL planar segmentation includes
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

       
ros::Publisher pub;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // ... do data processing
  sensor_msgs::PointCloud2 output;

  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg (*input, cloud);

  pcl::ModelCoefficients coefficients;
  pcl::PointIndices inliers;

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients(true);
  // Mandatory
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.01);

  seg.setInputCloud(cloud.makeShared());
  seg.segment(inliers, coefficients);

  if (inliers.indices.size () == 0)
  {
    ROS_ERROR ("Could not estimate a planar model for the given dataset.");
  // return (-1);
  }

  std::cerr << "Model coefficients: " << coefficients.values[0] << " " 
    << coefficients.values[1] << " "
    << coefficients.values[2] << " " 
    << coefficients.values[3] << std::endl;

  std::cerr << "Model inliers: " << inliers.indices.size () << std::endl;
  for (size_t i = 0; i < inliers.indices.size (); ++i)
    std::cerr << inliers.indices[i] << "    " << cloud.points[inliers.indices[i]].x << " "
      << cloud.points[inliers.indices[i]].y << " "
      << cloud.points[inliers.indices[i]].z << std::endl;

  // Publish the data
  pub.publish (coefficients);

}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pcl_planarSegmentation");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("points2", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  //pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  // Create a ROS publisher for the output model coefficients
  pub = nh.advertise<pcl::ModelCoefficients> ("output", 1);

  // Spin
  ros::spin ();
}
