#ifndef UTILS_PANGOCLOUD_H_
#define UTILS_PANGOCLOUD_H_

#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

// Because PCL and Pangolin don't play nice
#ifdef HAVE_OPENNI
#undef HAVE_OPENNI
#endif

#ifdef HAVE_OPENNI2
#undef HAVE_OPENNI2
#endif

#include "System.h"
#include "KeyFrame.h"
#include <pangolin/pangolin.h>
#include <pangolin/gl/gl.h>
#include <pangolin/gl/gldraw.h>
#include <mutex>

namespace ORB_SLAM2{

class DenseCloud{
  public:
    typedef pcl::PointXYZRGBA PointT;
    typedef pcl::PointCloud<PointT> PointCloud;
    
    DenseCloud(pcl::PointCloud<pcl::PointXYZRGB> *cloud); 
    DenseCloud(pcl::PointCloud<pcl::PointXYZRGBNormal> *cloud);
    
    virtual ~DenseCloud(){ glDeleteBuffers(1, &vbo); }

    void drawPoints();

    PointCloud::Ptr generatePointCloud(KeyFrame *kf, cv::Mat &color, cv::Mat &depth);
  private:

    // data to generate the point clouds
    std::vector<KeyFrame*>       keyFrames;
    std::vector<cv::Mat>         colorImgs;
    std::vector<cv::Mat>         depthImgs;

    // 多线程用来控制获得相应的data
    std::mutex              keyFrameMutex;
    uint16_t                lastKeyFrameSize = 0;

    double resolution = 0.04;
    pcl::VoxelGrid<pcl::PointXYZRGBA> voxel;

    const int numPoints;
    const int offset;
    const int stride;
    GLuint vbo;
  };
}
#endif
