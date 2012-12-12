#include "beginner_tutorials/pcl_extractor.h" 

PclExtractor::PclExtractor(ros::NodeHandle nh, ros::NodeHandle nh_private): 
  nh_(nh),
  nh_private_(nh_private)
{
  ROS_INFO("Starting pclExtractor");
  // register dynamic reconfigure
  PclExtractorConfigServer::CallbackType f = boost::bind(&PclExtractor::reconfigCallback, this, _1, _2);
  config_server_.setCallback(f);
  // get parameters
 // nh_private_.param("minPoints", minPoints_, 100);
  if(!nh_private_.getParam("minPoints", minPoints_))
    minPoints_ = 100;  
  if(!nh_private_.getParam("seg_setEpsAngle", seg_setEpsAngle_))
    seg_setEpsAngle_= 0.1;
 // if(!nh_private_.getParam("seg_setMethodType", seg_setMethodType_))
 //   seg_setMethodType_ = 1; 
  if(!nh_private_.getParam("seg_setMaxIteration", seg_setMaxIterations_))
    seg_setMaxIterations_ = 100; 
  if(!nh_private_.getParam("seg_setDistanceThreshold", seg_setDistanceThreshold_))
    seg_setDistanceThreshold_ = 0.25; 
  if(!nh_private_.getParam("chull_setDimension", chull_setDimension_))
    chull_setDimension_ = 2; 
  if(!nh_private_.getParam("sor_setMeanK", sor_setMeanK_))
    sor_setMeanK_ = 50;
  if(!nh_private_.getParam("sor_setStddevMulThresh", sor_setStddevMulThresh_))
    sor_setStddevMulThresh_ = 1.0;
  if(!nh_private_.getParam("seg_setNomalDistanceWeight", seg_setNormalDistanceWeight_))
    seg_setDistanceThreshold_ = 0.1; 
//  nh_private_.param("fixed_frame", fixed_frame_, "base_link");

  // register publishers
  hull_pub_       = nh_.advertise<sensor_msgs::PointCloud2> ("cloud/convex_hull", 1) ;
  object_pub_     = nh_.advertise<sensor_msgs::PointCloud2> ("cloud/object", 1) ;
  nonObject_pub_  = nh_.advertise<sensor_msgs::PointCloud2> ("cloud/nonObject", 1) ;
  plane_pub_      = nh_.advertise<sensor_msgs::PointCloud2> ("cloud/plane", 1) ;

  // register subscribers
  cloud_sub_      = nh_.subscribe("points2", 1, &PclExtractor::cloudCallback, this);

  // register dynamic reconfigure
 // PclExtractorConfigServer::CallbackType f = boost::bind(&PclExtractor::reconfigCallback, this, _1, _2);
 // config_server_.setCallback(f);
  
}

PclExtractor::~PclExtractor()
{
  ROS_INFO("Destroying PclExtractor()");
}

void PclExtractor::cloudCallback(const Cloud2Msg::ConstPtr& cloud2Msg_input)
{

  boost::mutex::scoped_lock(mutex_);
  sensor_msgs::PointCloud2 output;
  sensor_msgs::PointCloud2 convex_hull;
  sensor_msgs::PointCloud2 object_msg;
  sensor_msgs::PointCloud2 nonObject_msg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg (*cloud2Msg_input, *cloud);


  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  
  // *** Normal estimation
  // Create the normal estimation class and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(cloud);
  // Creating the kdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  ne.setSearchMethod(tree);

  // output dataset
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

  // use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch(0.3);

  // compute the features
  ne.compute(*cloud_normals);
  // *** End of normal estimation
  // *** Plane Estimation From Normals Start
  // Create the segmentation object
  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
  // Optional
  seg.setOptimizeCoefficients(true);
  // Mandatory
//  seg.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
  seg.setModelType(pcl::SACMODEL_PLANE);
//  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  //y가 z
//  const Eigen::Vector3f z_axis(0,-1.0,0);
//  seg.setAxis(z_axis);
//  seg.setEpsAngle(seg_setEpsAngle_);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations (seg_setMaxIterations_);
  seg.setDistanceThreshold(seg_setDistanceThreshold_);
  seg.setNormalDistanceWeight(seg_setNormalDistanceWeight_);
//  seg.setProbability(seg_probability_);

  seg.setInputCloud((*cloud).makeShared());
  seg.setInputNormals(cloud_normals);
  seg.segment(*inliers, *coefficients);

  std::cerr <<"input: "<<cloud->width*cloud->height<<"Model inliers: " << inliers->indices.size () << std::endl;
  if (inliers->indices.size () == 0)
  {
    ROS_ERROR ("Could not estimate a planar model for the given dataset.");
  }
  // *** End of Plane Estimation
  // *** Plane Estimation Start
  // Create the segmentation object
/*  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  //seg.setOptimizeCoefficients(true);
  // Mandatory
  seg.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
//  seg.setModelType(pcl::SACMODEL_PLANE);
//  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  //y가 z
  const Eigen::Vector3f z_axis(0,-1.0,0);
  seg.setAxis(z_axis);
  seg.setEpsAngle(seg_setEpsAngle_);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations (seg_setMaxIterations_);
  seg.setDistanceThreshold(seg_setDistanceThreshold_);

  seg.setInputCloud((*cloud).makeShared());
  seg.segment(*inliers, *coefficients);

  std::cerr <<"input: "<<cloud->width*cloud->height<<"Model inliers: " << inliers->indices.size () << std::endl;
  if (inliers->indices.size () == 0)
  {
    ROS_ERROR ("Could not estimate a planar model for the given dataset.");
  }
  // *** End of Plane Estimation
*/
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  // Extrat the inliers
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);

  pcl::PointCloud<pcl::PointXYZ>::Ptr ground_hull(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);

  if ((int)inliers->indices.size() > minPoints_)
  {
    extract.setNegative(false);
    extract.filter(*cloud_p);

    pcl::toROSMsg(*cloud_p, output);
    std::cerr << "PointCloud representing the planar component: " 
      << cloud_p->width * cloud_p->height << " data points." << std::endl;

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
    chull.setDimension(chull_setDimension_);//2D
    chull.reconstruct(*ground_hull);
    pcl::toROSMsg(*ground_hull, convex_hull);
    //pcl::toROSMsg(*ground_hull, convex_hull);
    ROS_INFO ("Convex hull has: %d data points.", (int) ground_hull->points.size ());

    // Publish the data
    plane_pub_.publish (output);
    hull_pub_.publish(convex_hull);

  }
  // Create the filtering object
  extract.setNegative(true);
  extract.filter(*cloud_f);
  ROS_INFO ("Cloud_f has: %d data points.", (int) cloud_f->points.size ());
  // cloud.swap(cloud_f);

  pcl::PointCloud<pcl::PointXYZ>::Ptr object(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr nonObject(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointIndices::Ptr object_indices(new pcl::PointIndices);

  // cloud statictical filter
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudStatisticalFiltered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud_f);
  sor.setMeanK(sor_setMeanK_);
  sor.setStddevMulThresh(sor_setStddevMulThresh_);
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
  object_pub_.publish(object_msg);
  //pcl::toROSMsg(*nonObject, nonObject_msg);
  //pcl::toROSMsg(*cloud_f, nonObject_msg);
  pcl::toROSMsg(*cloudStatisticalFiltered, nonObject_msg);
  nonObject_pub_.publish(nonObject_msg);
}


void PclExtractor::reconfigCallback(PclExtractorConfig& config, uint32_t level)
{
  boost::mutex::scoped_lock(mutex_);
  minPoints_ = config.minPoints;
  seg_setEpsAngle_ = config.seg_setEpsAngle;
  seg_setDistanceThreshold_ = config.seg_setDistanceThreshold;
  ROS_INFO("Dynamic reconfigure: %d, %f, %f", minPoints_, seg_setEpsAngle_, seg_setDistanceThreshold_);
}

