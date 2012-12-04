#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// PCL Euclidean cluuster extration
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
       
ros::Publisher pub;
ros::Publisher object_pub;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // ... do data processing
  sensor_msgs::PointCloud2 output;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg (*input, *cloud);

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.10); // 2cm
  ec.setMinClusterSize (20);
  ec.setMaxClusterSize (2500);
  ec.setSearchMethod (tree);
  ec.setInputCloud(cloud);
  ec.extract (cluster_indices);
  
  ROS_INFO("cluster_indices has %d data points.", (int) cluster_indices.size());
  ROS_INFO("cloud has %d data points.", (int) cloud->points.size());

  visualization_msgs::Marker marker;
  marker.header   = cloud->header;
  marker.id       = 1;
  marker.type     = visualization_msgs::Marker::CUBE_LIST;
  marker.action   = visualization_msgs::Marker::ADD;
  marker.color.r  = 1;
  marker.color.g  = 0;
  marker.color.b  = 0;
  marker.color.a  = 0.7;
  marker.scale.x  = 0.2;
  marker.scale.y  = 0.2;
  marker.scale.z  = 0.2;
  marker.lifetime = ros::Duration(60.0);
  Eigen::Vector4f minPoint;
  Eigen::Vector4f maxPoint;
//  pcl::getMinMax3D(*cloud, minPoint, maxPoint);

  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
      cloud_cluster->points.push_back (cloud->points[*pit]); 

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    // Merge current clusters to whole point cloud
    *clustered_cloud += *cloud_cluster;
 //   for(size_t j = 0; j < cloud_cluster->points.size() - 1; j++)
 //   {
      /*
      geometry_msgs::Point pt1;
      pt1.x = cloud_cluster->points[j].x;
      pt1.y = cloud_cluster->points[j].y;
      pt1.z = cloud_cluster->points[j].z;
      geometry_msgs::Point pt2;
      pt2.x = cloud_cluster->points[j+1].x;
      pt2.y = cloud_cluster->points[j+1].y;
      pt2.z = cloud_cluster->points[j+1].z;

      marker.points.push_back(pt1);
      marker.points.push_back(pt2);
      */
      //Seg for top of prism
      geometry_msgs::Point pt3;      
      pt3.x = 0.0;
      pt3.y = 0.0;
      pt3.z = 0.0;
      std_msgs::ColorRGBA colors;
      colors.r = 0.0;
      colors.g = 0.0;
      colors.b = 0.0;
      for(size_t i=0; i<cloud_cluster->points.size(); i++)
      {
        pt3.x += cloud_cluster->points[i].x;
        pt3.y += cloud_cluster->points[i].y;
        pt3.z += cloud_cluster->points[i].z;
      }
      pt3.x /= cloud_cluster->points.size();
      pt3.y /= cloud_cluster->points.size();
      pt3.z /= cloud_cluster->points.size();
      pcl::getMinMax3D(*cloud_cluster, minPoint, maxPoint);
      marker.scale.y= maxPoint.y();
      //marker.scale.x= maxPoint.x();
      //marker.scale.z= maxPoint.z();
      marker.scale.x= maxPoint.x()-minPoint.x();
      colors.r = marker.scale.x;
//      colors.g = marker.scale.y;
      //marker.scale.z= maxPoint.z()-minPoint.z();
      //pt3.z = maxPoint.z();

      //geometry_msgs::Point pt4;
      //pt4.x = cloud_cluster->points[j+1].x;
      //pt4.y = cloud_cluster->points[j+1].y;
      //pt4.z = cloud_cluster->points[j+1].z;
      //pt4.z = maxPoint.z();

      //marker.pose.position.x = pt3.x;
      //marker.pose.position.y = pt3.y;
      //marker.pose.position.z = pt3.z;
      //marker.colors.push_back(colors);
      marker.points.push_back(pt3);
      //marker.points.push_back(pt4);

      //Seg for bottom vertices to top vertices
     // marker.points.push_back(pt1);
      //marker.points.push_back(pt3);
 //   }
    object_pub.publish(marker);

  }
  // Publish the data
  pcl::toROSMsg(*clustered_cloud, output);
  output.header = cloud->header;
//  output.header.frame_id="/camera_link";
//  output.header.stamp = ros::Time::now();

  pub.publish (output);

}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pcl_euclideanClusterExtration");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("points2", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  //pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  // Create a ROS publisher for the output model coefficients
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
  object_pub = nh.advertise<visualization_msgs::Marker> ("object_marker", 1);

  // Spin
  ros::spin ();
}
