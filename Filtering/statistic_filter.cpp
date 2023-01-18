
#include <iostream> 
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h> 
#include <pcl/point_cloud.h>
#include <boost/random.hpp>
#include <pcl/console/time.h>
#include <pcl/filters/convolution_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>   //统计滤波器头文件


using namespace std;



int
main (int argc, char** argv)
{

  pcl::PointCloud<pcl::PointXYZ>::Ptr inputcloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_StatisticalRemoval(new pcl::PointCloud<pcl::PointXYZ>);


  pcl::PCDReader reader;
  pcl::PCDWriter writer;
  reader.read ("/home/zuqing/Documents/Git/PCL_learning/office/demo.pcd", *inputcloud);

 
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> Statistical;
  Statistical.setInputCloud(inputcloud);
  Statistical.setMeanK(20);//取平均值的临近点数
  Statistical.setStddevMulThresh(5);//临近点数数目少于多少时会被舍弃
  Statistical.filter(*cloud_after_StatisticalRemoval);


  writer.write<pcl::PointXYZ> ("/home/zuqing/Documents/Git/PCL_learning/office/demo_statistic_remove_filter.pcd", *cloud_after_StatisticalRemoval, false);
  std::cout << "finish" << std::endl;
 

}