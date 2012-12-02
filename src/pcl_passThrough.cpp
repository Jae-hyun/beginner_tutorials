#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// PCL voxel include
#include <pcl/filters/passthrough.h>

ros::Publisher pub;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // ... do data processing
  sensor_msgs::PointCloud2 output;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg (*input, *cloud);


  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

  std::cerr << "Before Path Thorugh filtering: "
    << cloud->width * cloud->height << std::endl;
  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZ> pass;
  // X-axis pass through filter
  pass.setInputCloud (cloud);
  // 키넥트는 x=x y=-z z=y
  // z축은 화면에 보이는것과 반대로 설정되어 있으니 반대로 설정해야 함
  pass.setFilterFieldName ("x"); //width
  pass.setFilterLimits (-2.5,2.5);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud);

  // X-axis pass through filter
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("y"); //height
  pass.setFilterLimits (-0.1,1.0);
  pass.filter (*cloud);

  // X-axis pass through filter
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z"); //length
  pass.setFilterLimits (0,5.0);
  pass.filter (*cloud_filtered);

  std::cerr << "After Path Thorugh filtering: "
    << cloud_filtered->width * cloud_filtered->height << std::endl;
  /*
  for (size_t i = 0; i < 10; ++i)
    std::cerr << "    " << cloud_filtered->points[i].x << " " 
      << cloud_filtered->points[i].y << " " 
      << cloud_filtered->points[i].z << std::endl;
*/
  pcl::toROSMsg(*cloud_filtered, output);
   
  // Publish the data
  pub.publish (output);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pcl_passThroughFilter");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("points2", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  // Create a ROS publisher for the output model coefficients
  //pub = nh.advertise<pcl::ModelCoefficients> ("output", 1);

  // Spin
  ros::spin ();
}
