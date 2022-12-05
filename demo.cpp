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
  std::cout << "passed through" << std::endl;
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

  // writer.write<pcl::PointXYZRGB> ("/home/zuqing/Documents/Git/PCL_learning/office/demo_xy_removeGround.pcd", *cloud_filtered_xy, false);

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
  std::shared_ptr<Filter> myFilter = std::make_shared<Filter>();
  // Show help
  if (pcl::console::find_switch (argc, argv, "-h") || pcl::console::find_switch (argc, argv, "--help")) {
    showHelp (argv[0]);
    return 0;
  }

  // Fetch point cloud filename in arguments | Works with PCD and PLY files
  std::vector<int> filenames;
  bool file_is_pcd = false;

  filenames = pcl::console::parse_file_extension_argument (argc, argv, ".ply");

  if (filenames.size () != 1)  {
    filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");

    if (filenames.size () != 1) {
      showHelp (argv[0]);
      return -1;
    } else {
      file_is_pcd = true;
    }
  }

  // Load file | Works with PCD and PLY files
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());

  if (file_is_pcd) {
    if (pcl::io::loadPCDFile (argv[filenames[0]], *source_cloud) < 0)  {
      std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
      showHelp (argv[0]);
      return -1;
    }
  } else {
    if (pcl::io::loadPLYFile (argv[filenames[0]], *source_cloud) < 0)  {
      std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
      showHelp (argv[0]);
      return -1;
    }
  }

  if (source_cloud->isOrganized()){
      printf ("\nworks\n");
      std::cout << "it is organized and the height is: " << source_cloud->height << std::endl << std::endl;
  }
  std::cout << "source cloud point num =" << source_cloud->size() << " \n";

  myFilter->setPointCloud(source_cloud);
  myFilter->passThrough();
/*
  passthough
*/

  pcl::PCDWriter writer;
/*
 *  voxel 0.1
 */
  pcl::PCLPointCloud2::Ptr PCLcloud (new pcl::PCLPointCloud2 ());
  pcl::PCLPointCloud2::Ptr cloud_filtered_voxel (new pcl::PCLPointCloud2 ());
  pcl::PCDReader reader;
  reader.read ("/home/zuqing/Documents/Git/PCL_learning/office/demo.pcd", *PCLcloud); 
  std::cout << "original point num = " << PCLcloud->data.size() << " \n";
  auto start = std::chrono::high_resolution_clock::now();
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor_1;
  sor_1.setInputCloud (PCLcloud);
  sor_1.setLeafSize (0.1f, 0.1f, 0.1f);
  sor_1.filter (*cloud_filtered_voxel);
  std::chrono::duration<double> used = std::chrono::high_resolution_clock::now() - start;
  std::cout << "0.1 voxel used time: " << used.count() << "s,  ";
  std::cout << "data size is  =" << cloud_filtered_voxel->data.size() << " \n";
  // writer.write ("/home/zuqing/Documents/Git/PCL_learning/office/demo_voxel_0.1.pcd", *cloud_filtered_voxel, 
        //  Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);
/*
 *  voxel  0.05 
*/  
  start = std::chrono::high_resolution_clock::now();
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor_2;
  sor_2.setInputCloud (PCLcloud);
  sor_2.setLeafSize (0.05f, 0.05f, 0.05f);
  sor_2.filter (*cloud_filtered_voxel);
  used = std::chrono::high_resolution_clock::now() - start;
  std::cout << "0.05 voxel used time: " << used.count() << " s,  ";
  std::cout << "data size is = " << cloud_filtered_voxel->data.size() << " \n";
/*
 *  voxel  0.01 
*/  
  start = std::chrono::high_resolution_clock::now();
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor_3;
  sor_3.setInputCloud (PCLcloud);
  sor_3.setLeafSize (0.05f, 0.05f, 0.05f);
  sor_3.filter (*cloud_filtered_voxel);
  used = std::chrono::high_resolution_clock::now() - start;
  std::cout << "0.01 voxel used time: " << used.count() << " s,  ";
  std::cout << "data size is  =" << cloud_filtered_voxel->data.size() << " \n";

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_projected_x (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_projected_y (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_projected_z (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  
  // x
  start = std::chrono::high_resolution_clock::now();
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
  used = std::chrono::high_resolution_clock::now() - start;
  std::cout << "project_x used time: " << used.count() << " s\n";
  // writer.write<pcl::PointXYZRGB> ("/home/zuqing/Documents/Git/PCL_learning/office/demo_projected_x.pcd", *cloud_projected_x, false);
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
  // writer.write<pcl::PointXYZRGB> ("/home/zuqing/Documents/Git/PCL_learning/office/demo_projected_y.pcd", *cloud_projected_y, false);
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
  // writer.write<pcl::PointXYZRGB> ("/home/zuqing/Documents/Git/PCL_learning/office/demo_projected_z.pcd", *cloud_projected_z, false);


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
  // writer.write<pcl::PointXYZRGB> ("/home/zuqing/Documents/Git/PCL_learning/office/demo_transformed.pcd", *transformed_cloud, false);
  

/*
  reove the outlier
*/
  pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr outliered_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
  // build the filter
  start = std::chrono::high_resolution_clock::now();
  outrem.setInputCloud(source_cloud);
  outrem.setRadiusSearch(0.05);
  outrem.setMinNeighborsInRadius (5);
  // outrem.setKeepOrganized(true);
  // apply filter
  outrem.filter (*outliered_cloud);
  used = std::chrono::high_resolution_clock::now() - start;
  std::cout << "k-min outlier filter used time: " << used.count() << " s,   ";
  std::cout << "with point num =  " << outliered_cloud->size() << " \n";
  // writer.write<pcl::PointXYZRGB> ("/home/zuqing/Documents/Git/PCL_learning/office/demo_outliered.pcd", *outliered_cloud, false);


/*
  filter outlier with condition
*/
    // build the condition
  start = std::chrono::high_resolution_clock::now();
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr condiitoned_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond (new
    pcl::ConditionAnd<pcl::PointXYZRGB> ());
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("y", pcl::ComparisonOps::GT, -4.0)));
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("y", pcl::ComparisonOps::LT, 0.5)));
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("x", pcl::ComparisonOps::GT, -4.0)));
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("x", pcl::ComparisonOps::LT, 4.0))); 
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("z", pcl::ComparisonOps::GT, 0.3)));
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("z", pcl::ComparisonOps::LT, 8.3)));
  // build the filter
  pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem;
  condrem.setCondition (range_cond);
  condrem.setInputCloud (source_cloud);
  // condrem.setKeepOrganized(true);
  // apply filter
  condrem.filter (*condiitoned_cloud);
  used = std::chrono::high_resolution_clock::now() - start;
  std::cout << "condition filter used time: " << used.count() << " s,   ";
  std::cout << "with point num =  " << condiitoned_cloud->size() << " \n";
  writer.write<pcl::PointXYZRGB> ("/home/zuqing/Documents/Git/PCL_learning/office/demo_condition_rem.pcd", *condiitoned_cloud, false);

  /*
  visualization
  */
  pcl::visualization::CloudViewer viewer("Cloud Viewer");
  viewer.showCloud(condiitoned_cloud);
	while (!viewer.wasStopped())
	{
 
	}
  return 0;
}