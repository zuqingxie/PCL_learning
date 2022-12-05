#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h> // here include the transform function  pcl::transformPointCloud
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/project_inliers.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
class Filter{
public:
  void passThrough();
  void setPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud);
private:
  pcl::PCDWriter writer;
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud;

};

void Filter::passThrough(){
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_xy (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  // pcl::PassThrough<pcl::PointXYZRGB> pass_y;
  auto start = std::chrono::high_resolution_clock::now();
  pass.setInputCloud (source_cloud);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (0.0, 100.0);
  // pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered_xy);

  pass.setFilterFieldName ("y");
  pass.setFilterLimits (-4.0, 0.5);
  pass.filter (*cloud_filtered_xy);

  std::chrono::duration<double> used = std::chrono::high_resolution_clock::now() - start;
  std::cout << "passthrough used time: " << used.count() << " s\n";

  writer.write<pcl::PointXYZRGB> ("/home/zuqing/Documents/Git/PCL_learning/office/demo_voxel_0.1/demo_xy_removeGround.pcd", *cloud_filtered_xy, false);

};
void Filter::setPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr s_cloud){
  source_cloud =s_cloud;
  std::cout << "sucessfully set the source_cloud" << std::endl;
};
// This function displays the help
void showHelp(char * program_name)
{
  std::cout << std::endl;
  std::cout << "Usage: " << program_name << " cloud_filename.[pcd|ply]" << std::endl;
  std::cout << "-h:  Show this help." << std::endl;
};



// This is the main function
int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());

  // Show help
  std::string source_file = "/home/zuqing/Documents/Git/PCL_learning/office/demo_voxel_0.07/demo_voxel_0.07.pcd";
  std::string save_path = "/home/zuqing/Documents/Git/PCL_learning/office/demo_voxel_0.07/";
  pcl::PCDReader reader;
  pcl::PCDWriter writer;


  reader.read (source_file, *source_cloud); 
  std::cout << "original point num = " << source_cloud->size() << " \n";


/*
  passthough
*/
  std::shared_ptr<Filter> myFilter = std::make_shared<Filter>();
  myFilter->setPointCloud(source_cloud);
  myFilter->passThrough();




/*
  project
*/
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_projected_x (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_projected_y (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_projected_z (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  
  // x
  auto start = std::chrono::high_resolution_clock::now();
  coefficients->values.resize (4);
  coefficients->values[0] = 1.0;
  coefficients->values[1] = 0;
  coefficients->values[2] = 0;
  coefficients->values[3] = 0;
  pcl::ProjectInliers<pcl::PointXYZRGB> proj_x;
  proj_x.setModelType (pcl::SACMODEL_PLANE);
  proj_x.setInputCloud (source_cloud);
  proj_x.setModelCoefficients (coefficients);
  proj_x.filter (*cloud_projected_x);
  std::chrono::duration<double> used = std::chrono::high_resolution_clock::now() - start;
  std::cout << "project_x used time: " << used.count() << " s\n";
  writer.write<pcl::PointXYZRGB> (save_path + "demo_projected_x.pcd", *cloud_projected_x, false);
  // y

  start = std::chrono::high_resolution_clock::now();

  coefficients->values[0] = 0;
  coefficients->values[1] = 1.0;
  coefficients->values[2] = 0;
  coefficients->values[3] = 0;
  pcl::ProjectInliers<pcl::PointXYZRGB> proj_y;
  proj_y.setModelType (pcl::SACMODEL_PLANE);
  proj_y.setInputCloud (source_cloud);
  proj_y.setModelCoefficients (coefficients);
  proj_y.filter (*cloud_projected_y);
  used = std::chrono::high_resolution_clock::now() - start;
  std::cout << "project_y used time: " << used.count() << " s\n";
  writer.write<pcl::PointXYZRGB> (save_path + "demo_projected_y.pcd", *cloud_projected_y, false);
  // z

  start = std::chrono::high_resolution_clock::now();

  coefficients->values[0] = 0;
  coefficients->values[1] = 0;
  coefficients->values[2] = 1.0;
  coefficients->values[3] = 0;
  pcl::ProjectInliers<pcl::PointXYZRGB> proj_z;
  proj_z.setModelType (pcl::SACMODEL_PLANE);
  proj_z.setInputCloud (source_cloud);
  proj_z.setModelCoefficients (coefficients);
  proj_z.filter (*cloud_projected_z);
  used = std::chrono::high_resolution_clock::now() - start;
  std::cout << "project_z used time: " << used.count() << " s\n";
  writer.write<pcl::PointXYZRGB> (save_path + "demo_projected_z.pcd", *cloud_projected_z, false);


  /*
  transformation
  */

  start = std::chrono::high_resolution_clock::now();
  Eigen::Affine3f transformMatrix = Eigen::Affine3f::Identity();
  float theta = M_PI/2;
  // Define a translation of 2.5 meters on the x axis.
  transformMatrix.translation() << 0.0, 0.0, 0.0;
  // The same rotation matrix as before; theta radians around Z axis
  transformMatrix.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));
  // Print the transformation
  // std::cout << transformMatrix.matrix() << std::endl;
  // Executing the transformation
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
  // You can either apply transform_1 or transform_2; they are the same
  pcl::transformPointCloud (*source_cloud, *transformed_cloud, transformMatrix);
  used = std::chrono::high_resolution_clock::now() - start;
  std::cout << "transform used time: " << used.count() << " s\n";
  writer.write<pcl::PointXYZRGB> (save_path + "demo_transformed.pcd", *transformed_cloud, false);
  
  /*
  visualization
  */
  
  pcl::visualization::CloudViewer viewer("Cloud Viewer");
  viewer.showCloud(source_cloud);
	while (!viewer.wasStopped())
	{
 
	}
  return 0;
}