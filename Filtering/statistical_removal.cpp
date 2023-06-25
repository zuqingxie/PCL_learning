#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
  // Fill in the cloud data
  pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  reader.read<pcl::PointXYZRGB> ("/home/zuqing/Documents/Git/PCL_learning/office/demo.pcd", *cloud);

  std::cerr << "Cloud before filtering: " << std::endl;
  std::cerr << *cloud << std::endl;
  auto start = std::chrono::high_resolution_clock::now();

  // Create the filtering objectcode
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK (100); // 50 neighbors will be considered
  sor.setStddevMulThresh (0.1); // standard deviation multiplier to 0.8, smaller more will be removed
  sor.filter (*cloud_filtered);

  std::chrono::duration<double> used = std::chrono::high_resolution_clock::now() - start;
  std::cout << "statistical remove filter used time: " << used.count() << " s\n";
  std::cerr << *cloud_filtered << std::endl;



  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZRGB> ("/home/zuqing/Documents/Git/PCL_learning/office/demo_statistical_inliers.pcd", *cloud_filtered, false);

  sor.setNegative (true);
  sor.filter (*cloud_filtered);
  writer.write<pcl::PointXYZRGB> ("/home/zuqing/Documents/Git/PCL_learning/office/demo_statistical_outliers.pcd", *cloud_filtered, false);

  return (0);
}