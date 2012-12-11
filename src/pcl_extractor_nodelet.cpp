#include "beginner_tutorials/pcl_extractor_nodelet.h"

PLUGINLIB_DECLARE_CLASS(beginner_tutorials, PclExtractorNodelet, PclExtractorNodelet, nodelet::Nodelet);

void PclExtractorNodelet::onInit()
{
  NODELET_INFO("Initializing PCL Extractor Nodelet");  
 
  ros::NodeHandle nh         = getMTNodeHandle();
  ros::NodeHandle nh_private = getMTPrivateNodeHandle();

  extractor_  = new PclExtractor(nh, nh_private);
}
