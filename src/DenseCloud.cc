#include "DenseCloud.h"
#include "Converter.h"

namespace ORB_SLAM2{

  DenseCloud::DenseCloud(pcl::PointCloud<pcl::PointXYZRGB> *cloud)
    : numPoints(cloud->size()), offset(4), stride(sizeof(pcl::PointXYZRGB)){
      
      // 创建缓冲对象
      glGenBuffers(1, &vbo);
      
      // 激活缓冲对象(GL_ARRAY_BUFFER是坐标,颜色等)
      glBindBuffer(GL_ARRAY_BUFFER, vbo);

      // 创建内存, 并向当中传递数据存入其中
      glBufferData(GL_ARRAY_BUFFER, cloud->points.size()*stride, cloud->points.data(), GL_STATIC_DRAW);
      glBindBuffer(GL_ARRAY_BUFFER, 0);
    }


  DenseCloud::DenseCloud(pcl::PointCloud<pcl::PointXYZRGBNormal> *cloud)
    : numPoints(cloud->size()), offset(8), stride(sizeof(pcl::PointXYZRGBNormal)){
      glGenBuffers(1, &vbo);
      glBindBuffer(GL_ARRAY_BUFFER, vbo);
      glBufferData(GL_ARRAY_BUFFER, cloud->points.size() * stride, cloud->points.data(), GL_STATIC_DRAW);
      glBindBuffer(GL_ARRAY_BUFFER, 0);
    }


  pcl::PointCloud<DenseCloud::PointT>::Ptr DenseCloud::generatePointCloud(KeyFrame *kf){
    PointCloud::Ptr tmp(new PointCloud());
    // point cloud is null ptr
    for(int m = 0; m < kf->depthImg.rows; m+=3){
      for( int n = 0; n < kf->depthImg.cols; n+=3){
        float d = kf->depthImg.ptr<float>(m)[n];
        if(d < 0.01 || d > 10)
          continue;
        PointT p;
        p.z = d;
        p.x = (n - kf->cx) * p.z / kf->fx;
        p.y = (m - kf->cy) * p.z / kf->fy;
        
        p.b = kf->colorImg.ptr<uchar>(m)[n*3];
        p.g = kf->colorImg.ptr<uchar>(m)[n*3+1];
        p.r = kf->colorImg.ptr<uchar>(m)[n*3+2];

        tmp->points.push_back(p);
      }
    }

    Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat( kf->GetPose() );
    PointCloud::Ptr cloud(new PointCloud);
    pcl::transformPointCloud(*tmp, *cloud, T.inverse().matrix());
    cloud->is_dense = false;
    std::cout << "Generate point cloud for kf " << kf->mnId << ", size=" << cloud->points.size() << endl;
    return cloud;
  }

  void DenseCloud::drawPoints(){
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glVertexPointer(3, GL_FLOAT, stride, 0);
    glColorPointer(3, GL_UNSIGNED_BYTE, stride, (void *)(sizeof(float) *offset));

    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);

    glDrawArrays(GL_POINTS, 0, numPoints);

    glDisableClientState(GL_COLOR_ARRAY);
    glDisableClientState(GL_VERTEX_ARRAY);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
  }
} // End of the namespace of ORB_SLAM2
