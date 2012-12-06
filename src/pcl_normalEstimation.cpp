#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
// PCL Euclidean cluuster extration
#include <pcl/kdtree/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>

//ros::Publisher pub;
//ros::Publisher object_pub;

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pcl_normalEstimation");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
//  ros::Subscriber sub = nh.subscribe ("points2", 1, cloud_cb);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  // Create a ROS publisher for the output model coefficients
//  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
  pcl::PCDReader reader;
  reader.read("/home/jaehyun/ros_workspace/table_scene_lms400.pcd", *cloud);

  // Create the normal estimation class and pass the input datset to it
//  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(cloud);
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  ne.setSearchMethod(tree);

  //output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  //use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch(0.03);

  //compute the features
  ne.compute(*cloud_normals);

  pcl::PointCloud<pcl::PointNormal>::Ptr  pn (new pcl::PointCloud<pcl::PointNormal>);
  //
  pcl::copyPointCloud<pcl::PointXYZ, pcl::PointNormal> (*cloud, *pn);
  pcl::copyPointCloud<pcl::Normal, pcl::PointNormal> (*cloud_normals, *pn);

  pcl::io::savePCDFileASCII("normal_pcd.pcd", *pn);

  std::cout << "before: " << cloud->points.size() << "after: " << cloud_normals->points.size() <<std::endl;

//  object_pub = nh.advertise<visualization_msgs::Marker> ("object_marker", 1);


  // Spin
//  ros::spin ();
  return 0;
}
