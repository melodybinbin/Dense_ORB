#include "DenseCloud.h"

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


  void DenseCloud::drawPoints(){
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glVertexPointer(3, GL_FLOAT, stride, 0);
    glColorPointer(3, GL_UNSIGNED_BYTE, stride, (void *)(sizeof(float) *offset));

    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);

    glDrawArrays(GL_POINTS, 0, numPoints);

    glDisableClientState(GL_COLOR_ARRAY);
    glDIsableClientState(GL_VERTEX_ARRAY);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
  }
}
