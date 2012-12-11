#ifndef PCL_EXTRACTOR_H 
#define PCL_EXTRACTOR_H 

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <dynamic_reconfigure/server.h>
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

#include "beginner_tutorials/PclExtractorConfig.h"
//ros::Publisher pub;
//ros::Publisher hull_pub;
//ros::Publisher object_pub;
//ros::Publisher nonObject_pub;
//ros::Publisher plane_pub;

class PclExtractor
{
  typedef sensor_msgs::PointCloud2  Cloud2Msg;
  typedef beginner_tutorials::PclExtractorConfig      PclExtractorConfig;
  typedef dynamic_reconfigure::Server<PclExtractorConfig>   PclExtractorConfigServer;
  public:

    PclExtractor(ros::NodeHandle nh, ros::NodeHandle nh_private);
    virtual ~PclExtractor();

  private:

    // ROS related
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Publisher hull_pub_;
    ros::Publisher object_pub_;
    ros::Publisher nonObject_pub_;
    ros::Publisher plane_pub_;

    ros::Subscriber cloud_sub_;

    PclExtractorConfigServer config_server_;
    // parameters
    boost::mutex mutex_;
    int minPoints_;
    std::string fixed_frame_;
    // segmentation parameters
    double  seg_setEpsAngle_;
    float   seg_setMethodType_;
    int     seg_setMaxIterations_;
    double  seg_setDistanceThreshold_;

    int     chull_setDimension_;

    int     sor_setMeanK_;
    double  sor_setStddevMulThresh_;

    // status variables

    //member function

    void cloudCallback(const Cloud2Msg::ConstPtr& Cloud2Msg_input);
    void reconfigCallback(PclExtractorConfig& config, uint32_t level);
  
};

#endif //PCL_EXTRACTOR_H 
