#include "beginner_tutorials/pcl_extractor.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "PclExtractor");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  PclExtractor pcl_extractor(nh, nh_private);
  ros::spin();
  return 0;
}
