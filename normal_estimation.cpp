#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <chrono>


int 
main ()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
 pcl::io::loadPCDFile ("/home/zuqing/Documents/Git/PCL_learning/office/table_scene_lms400.pcd", *cloud);

  auto start = std::chrono::high_resolution_clock::now();
  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);

  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch (0.03);

  // Compute the features
  ne.compute (*cloud_normals);
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> used = finish - start;
  std::cout << "unorganized used time: " << used.count() << " s\n";
    
    
  // visualize normals
  pcl::visualization::PCLVisualizer viewer("PCL Viewer");
  viewer.setBackgroundColor (0.0, 0.0, 0.5);
  viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, cloud_normals);
  
  while (!viewer.wasStopped ())
  {
    viewer.spinOnce ();
  }

  // cloud_normals->size () should have the same size as the input cloud->size ()*
}