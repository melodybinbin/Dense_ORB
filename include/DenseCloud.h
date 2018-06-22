#ifndef UTILS_PANGOCLOUD_H_
#define UTILS_PANGOCLOUD_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Because PCL and Pangolin don't play nice
#ifdef HAVE_OPENNI
#undef HAVE_OPENNI
#endif

#ifdef HAVE_OPENNI2
#undef HAVE_OPENNI2
#endif

#include <pangolin/pangolin.h>
#include <pangolin/gl/gl.h>
#include <pangolin/gl/gldraw.h>

namespace ORB_SLAM2{

class DenseCloud{
  public:
    
    DenseCloud(pcl::PointCloud<pcl::PointXYZRGB> *cloud); 
    DenseCloud(pcl::PointCloud<pcl::PointXYZRGBNormal> *cloud);

    virtual ~DenseCloud(){ glDeleteBuffers(1, &vbo); }

    void drawPoints();

  private:

    // data to generate the point clouds
    vector<KeyFrame*>       keyframes;
    vector<cv::Mat>         colorImgs;
    vector<cv::Mat>         depthImgs;

    // 多线程用来控制获得相应的data
    mutex                   keyframeMutex;
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
