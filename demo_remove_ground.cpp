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



#include <rclcpp/rclcpp.hpp>
#include "math.h"


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>


static int sorter(const void* val1, const void* val2);




class RemoveGround{
public:
    RemoveGround(rclcpp::Clock::SharedPtr clock, rclcpp::Logger log);

    void loadParameters(Params params);
    void setPointCloud(const  pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_cloud);
    void removeGround();
    std::shared_ptr<Ordered_Cloud> Ordered_Cloud;
private:

    static const int ground_lines_in_segment = 100;
    typedef struct
    {
        double a; //slope
        double b; //intercept
        double r; //Coefficient of determination. describe how well is the regression
    }T_Regression_Val;

    typedef struct
    {
        int start;
        int end;
        T_Regression_Val line_params; 
    }T_Line;


    void Add_Points_To_Cloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud);
    void Evaluate_Y_Prototype_Point();
    void Apply_Ground_Point_By_Regression();
	void Apply_Averaging();
	void Remove_Outlier();

    RemoveGround::T_Regression_Val linreg(std::vector<pcl::PointXYZRGB>& input);

    rclcpp::Clock::SharedPtr clock;
    rclcpp::Logger logger = rclcpp::get_logger("");
    rclcpp::Time time_old;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_after_remove;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    RemGroundParams rem_ground_params;
    GridParams grid_params;
    pcl::PCDWriter writer;
    OrderedCloudParams order_cloud_params;

}; 







typedef struct
{
    bool outlier = true;
    pcl::PointXYZRGB point;
}T_Point;

typedef struct
{
    int len = 0;
    double average_height;
    int in_cloud;
    bool contains_non_ground;
    double ground_truth;

}T_Mesh;

class Ordered_Cloud
{
public:
  // Ordered_Cloud(int i,int j, int k);
  Ordered_Cloud();
  // int actual_segment;
  // int min_segment;
  // int seg;

  // get i.th seg, j.bin and k.th point in the grid
  inline T_Point* get(int i,int j, int k)
      {return &clouds[(i)*max_point_number*bin_num+(j)*max_point_number+(k)];};

  void incLen(int i, int j);
  int len(int i,int j);
  void setAverageHeight(int i,int j,double a_h);
  // void set_in_cloud(int i, int j,int cloud_num);
  // int get_in_cloud(int i,int j);
  double getAverageHeight(int i,int j);
  // inline T_Mesh *get_misc_pointer(){return &mesh[0];};
  // void set_non_ground(int i,int j);
  // bool contains_non_ground(int i,int j);
  void set_ground_trouth(int i,int j,double gt);
  double get_ground_trouth(int i,int j);
  
	
	// void cleanup();
private:
  rclcpp::Clock::SharedPtr clock;
  rclcpp::Logger logger = rclcpp::get_logger("");
  OrderedCloudParams ordered_cloud_params;


  T_Point *clouds;
  T_Mesh *meshes;

  int seg_num; // 8
  int bin_num;  // 6
  int max_point_number; // 6000

  inline T_Mesh* mesh(int i ,int j) {return &mesh[(i) * bin_num + (j)];};
};




// This is the main function
int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr result_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());

  // Show help
  std::string source_file = "/home/zuqing/Documents/Git/PCL_learning/office/demo_remove_ground/demo.pcd";
  std::string save_path = "/home/zuqing/Documents/Git/PCL_learning/office/demo_remove_ground/";
  pcl::PCDReader reader;
  pcl::PCDWriter writer;
  reader.read (source_file, *source_cloud); 
  std::cout << "original point num = " << source_cloud->size() << " \n";


  //parameters 

  int beams = 9;
  int dis_num = 10;
  int angle_total = 90;
  int dis_total = 9; 

/*
  segment all the point into grid
*/
  auto start = std::chrono::high_resolution_clock::now();
  double max_angle = -9999;
  double min_angle = 9999;

  for(int i = 0; i < source_cloud->points.size() - 1; i++)
  {
    float  x = source_cloud->points[i].x;
    float  y = source_cloud->points[i].y;
    float  z = source_cloud->points[i].z;

    double angle = RAD2DEG(atan2(x,z));
    

    if(angle < min_angle) min_angle = angle;
    if(angle > max_angle) max_angle = angle;



  }

  std::cout << "max angle : " <<  max_angle << " s\n";
  std::cout << "min angle : " <<  min_angle << " s\n";

  std::chrono::duration<double> used = std::chrono::high_resolution_clock::now() - start;
  std::cout << "segment used time: " << used.count() << " s\n";






  /*
  visualization
  */
  
  pcl::visualization::CloudViewer viewer("Cloud Viewer");
  viewer.showCloud(result_cloud);
	while (!viewer.wasStopped())
	{
 
	}
  return 0;
}