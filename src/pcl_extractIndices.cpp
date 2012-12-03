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
// PCL extract indices includes
#include <pcl/filters/extract_indices.h>
// PCL voxel include
//#include <pcl/filters/voxel_grid.h>
// PCL convex hull include
//#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
// PCL extract polygonal prism data
#include <pcl/segmentation/extract_polygonal_prism_data.h>
// PCL project inliers include 
#include <pcl/filters/project_inliers.h>
#include <geometry_msgs/PolygonStamped.h>
#include <pcl/filters/statistical_outlier_removal.h>

ros::Publisher pub;
ros::Publisher hull_pub;
ros::Publisher object_pub;
ros::Publisher nonObject_pub;
ros::Publisher plane_pub;
int minPoints = 100;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // ... do data processing
  sensor_msgs::PointCloud2 output;
  sensor_msgs::PointCloud2 convex_hull;
  sensor_msgs::PointCloud2 object_msg;
  sensor_msgs::PointCloud2 nonObject_msg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//  std::cerr << "t0" << std::endl;
  pcl::fromROSMsg (*input, *cloud);

//  std::cerr << "t1" << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  //seg.setOptimizeCoefficients(true);
  // Mandatory
  seg.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
//  seg.setModelType(pcl::SACMODEL_PLANE);
//  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  const Eigen::Vector3f z_axis(0,-1.0,0);
  seg.setAxis(z_axis);
  seg.setEpsAngle(0.05);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold(0.03);

//  std::cerr << "t2" << std::endl;
//  seg.setInputCloud(cloud.makeShared());
//  seg.segment(inliers, coefficients);

  // Create the extract filtering object
 // pcl::ExtractIndices<pcl::PointXYZ> extract;
  int i = 0, nr_points = (int)cloud->points.size();

  // While 30%of the original cloud is still there
 // while(cloud->points.size() > 0.3 * nr_points)
 // {
    
    seg.setInputCloud((*cloud).makeShared());
    seg.segment(*inliers, *coefficients);
  /*
    std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
      << coefficients->values[1] << " "
      << coefficients->values[2] << " " 
      << coefficients->values[3] << std::endl;
*/
    std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
  /*  for (size_t k = 0; k < inliers.indices.size (); ++k)
      std::cerr << inliers.indices[k] << "    " 
        << cloud.points[inliers.indices[k]].x << " "
        << cloud.points[inliers.indices[k]].y << " "
        << cloud.points[inliers.indices[k]].z << std::endl;
 */
    if (inliers->indices.size () == 0)
    {
      ROS_ERROR ("Could not estimate a planar model for the given dataset.");
      //break;
      // return -1;
    }

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    // Extrat the inliers
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_hull(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);
    if ((int)inliers->indices.size() > minPoints)
    {
      extract.setNegative(false);
      extract.filter(*cloud_p);

      pcl::toROSMsg(*cloud_p, output);
      std::cerr << "PointCloud representing the planar component: " 
        << cloud_p->width * cloud_p->height << " data points." << std::endl;
    
     // std::cerr << "cloud width: " << cloud_p->width
     //   <<"cloud height: " << cloud_p->height << std::endl;
     // Step 3c. Project the ground inliers
      pcl::ProjectInliers<pcl::PointXYZ> proj;
      proj.setModelType(pcl::SACMODEL_PLANE);
      proj.setInputCloud(cloud_p);
      proj.setModelCoefficients(coefficients);
      proj.filter(*cloud_projected);
     // Step 3d. Create a Convex Hull representation of the projected inliers
      pcl::ConvexHull<pcl::PointXYZ> chull;
      //chull.setInputCloud(cloud_p);
      chull.setInputCloud(cloud_projected);
      chull.setDimension(2);//2D
      chull.reconstruct(*ground_hull);
      pcl::toROSMsg(*ground_hull, convex_hull);
    //    pcl::ConvexHull<pcl::PointXYZ> chull;
      //    chull.setInputCloud (cloud_projected);
        //    chull.reconstruct (*cloud_hull);
   /*   pcl::ConcaveHull<pcl::PointXYZ> chull;
      chull.setInputCloud(cloud_p);
      chull.setAlpha(0.01);
      chull.reconstruct(*ground_hull);
      pcl::toROSMsg(*ground_hull, convex_hull);
   */   
   /*   // estimate the convex hull
      pcl::PointCloud<Point> plane_hull;
      hull.setInputCloud (plane_projected_ptr);
     hull.reconstruct (plane_hull);
     */
      //pcl::toROSMsg(*ground_hull, convex_hull);
      ROS_INFO ("Convex hull has: %d data points.", (int) ground_hull->points.size ());
      // Publish the data
      pub.publish (output);
      hull_pub.publish(convex_hull);

      if ( ground_hull->points.size() >= 3)
      {
        geometry_msgs::PolygonStamped poly;
        poly.header.stamp = input->header.stamp;
        poly.header.frame_id =  "camera_link";
        // here, i check the direction of points
        // polygon must have CLOCKWISE direction
        for ( size_t i = 0; i < ground_hull->points.size(); i++ )
        {
          geometry_msgs::Point32 p;
          p.x = ground_hull->points[i].x;
          p.y = ground_hull->points[i].y;
          p.z = ground_hull->points[i].z;
          poly.polygon.points.push_back(p);
        }

        plane_pub.publish(poly);
      }
      else
      {
        ROS_FATAL("too small points are estimated");
      }
    }
    // Create the filtering object
    extract.setNegative(true);
    extract.filter(*cloud_f);
    ROS_INFO ("Cloud_f has: %d data points.", (int) cloud_f->points.size ());
   // cloud.swap(cloud_f);
    i++;
    // segment those points that are in the polygonal prism
    pcl::PointCloud<pcl::PointXYZ>::Ptr object(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr nonObject(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointIndices::Ptr object_indices(new pcl::PointIndices);

    pcl::ExtractPolygonalPrismData<pcl::PointXYZ> ex;
    ex.setInputCloud (cloud_f);
    ex.setInputPlanarHull (ground_hull);
//    ex.setInputPlanarHull (cloud_projected);
    ex.setHeightLimits(-1.0, 1.0);
    ex.segment (*object_indices);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudStatisticalFiltered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_f);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloudStatisticalFiltered);

    pcl::ExtractIndices<pcl::PointXYZ> exObjectIndices;
    //exObjectIndices.setInputCloud(cloud_f);
    exObjectIndices.setInputCloud(cloudStatisticalFiltered);
    exObjectIndices.setIndices(object_indices);
    exObjectIndices.filter(*object);
    exObjectIndices.setNegative(true);
    exObjectIndices.filter(*nonObject);

    ROS_INFO ("Object has: %d data points.", (int) object->points.size ());
    pcl::toROSMsg(*object, object_msg);
    object_pub.publish(object_msg);
    //pcl::toROSMsg(*nonObject, nonObject_msg);
    //pcl::toROSMsg(*cloud_f, nonObject_msg);
    pcl::toROSMsg(*cloudStatisticalFiltered, nonObject_msg);
    nonObject_pub.publish(nonObject_msg);
    /*
    if ( plane_hull.points.size() >= 3)
    {
      geometry_msgs::PolygonStamped poly;
      poly.header.stamp = pcl_cloud_ptr->header.stamp;
      poly.header.frame_id =  _plane_frame_id;
      // here, i check the direction of points
      // polygon must have CLOCKWISE direction
      tf::Vector3 O(plane_hull.points[1].x,
          plane_hull.points[1].y,
          plane_hull.points[1].z);
      tf::Vector3 B(plane_hull.points[0].x,
          plane_hull.points[0].y,
          plane_hull.points[0].z);
      tf::Vector3 A(plane_hull.points[2].x,
          plane_hull.points[2].y,
          plane_hull.points[2].z);
      tf::Vector3 OA = A-O;
      tf::Vector3 OB = B-O;
      tf::Vector3 N = OA.cross(OB);
      double theta = N.angle(O);
      bool reversed = false;
      if ( theta < M_PI / 2.0)
        reversed = true;
      for ( size_t i = 0; i < transed_hull.points.size(); i++ )
      {
        geometry_msgs::Point32 p;
        if ( reversed )
        {
          size_t j = transed_hull.points.size() - i - 1;
          p.x = transed_hull.points[j].x;
          p.y = transed_hull.points[j].y;
          p.z = transed_hull.points[j].z;
        }
        else
        {
          p.x = transed_hull.points[i].x;
          p.y = transed_hull.points[i].y;
          p.z = transed_hull.points[i].z;
        }
        poly.polygon.points.push_back(p);
      }

      _plane_pub.publish(poly);
    }
    else
    {
      ROS_FATAL("too small points are estimated");
    }*/
//    hull_limiter.setHeightLimits(0, 0.3);
//    hull_limiter.segment(object_indices);

//    ExtractIndices<PointT> object_extractor;
//    object_extractor.setInputCloud(boost::make_shared<CloudT>(object_points));
//    object_extractor.setIndices(boost::make_shared<PointIndices>(object_indices));
//    object_extractor.filter(object_points);
//    pub.publish (output);
    //hull_pub.publish(convex_hull);
//  }
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pcl_extractIndices");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("points2", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
  // Create a ROS publisher for the convex hull point cloud
  hull_pub = nh.advertise<sensor_msgs::PointCloud2> ("convex_hull", 1);
  // Create a ROS publisher for the object point cloud
  object_pub = nh.advertise<sensor_msgs::PointCloud2> ("objects", 1);

  // Create a ROS publisher for the non-object point cloud
  nonObject_pub = nh.advertise<sensor_msgs::PointCloud2> ("nonObjects", 1);
  plane_pub = nh.advertise<geometry_msgs::PolygonStamped>("estimated_plane", 10);


  // Create a ROS publisher for the output model coefficients
  //pub = nh.advertise<pcl::ModelCoefficients> ("output", 1);

  // Spin
  ros::spin ();
}
