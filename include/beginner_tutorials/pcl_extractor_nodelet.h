#ifndef PCL_EXTRACTOR_NODELET_H 
#define PCL_EXTRACTOR_NODELET_H 

// nodelet include files
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

// pcl_extractor include file
#include "beginner_tutorials/pcl_extractor.h"

class PclExtractorNodelet : public nodelet::Nodelet
{
  public:

    virtual void onInit();

  private:

    PclExtractor * extractor_;
  
};

#endif //PCL_EXTRACTOR_NODELET_H 
