#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;



int main(int argc, char** argv)
{
  ros::init (argc, argv, "pcl_pcdReadPublisher");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<PointCloud> ("points2", 1);

  PointCloud::Ptr msg(new PointCloud);
  /*
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
  */
  /*
  msg->header.frame_id = "some_tf_frame";
  msg->height = 15;
  msg->width = 15;
  msg->points.resize(msg->width * msg->height);
  //msg->points.push_back (pcl::PointXYZ(1.0, 2.0, 3.0));
  for(size_t i = 0; i < msg->points.size(); ++i)
  {
    msg->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    msg->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    msg->points[i].z = 0.01 + msg->points[i].x ;
   // msg->points[i].z = 0;
    ROS_INFO("%dth x y z : %.4f %.4f %.4f",(int)i, msg->points[i].x, msg->points[i].y, msg->points[i].z);
  }
  msg->points[1].z = 0.5; 
  msg->points[5].z = -0.5; 
  msg->points[10].z = 0.15; 
  msg->points[15].z = -0.15; 
  msg->points[20].z = 0.50; 
  msg->points[25].z = -0.50; 
  msg->points[30].z = 0.55; 
*/
  pcl::PCDReader reader;
  reader.read("/home/jhp/ros_workspace/table_scene_lms400.pcd", *msg);
  msg->header.frame_id = "base_link";
  int pointNum = msg->points.size();
  ROS_INFO("# of Points : %d\n", pointNum);
  ros::Rate loop_rate(10);

  while(nh.ok())
  {
    msg->header.stamp = ros::Time::now();
    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
}

