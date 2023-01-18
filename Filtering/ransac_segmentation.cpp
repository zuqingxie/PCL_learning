#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>


int 
main ()
{
    // Read in the point cloud data
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile("/home/zuqing/Documents/Git/PCL_learning/office/demo_voxel_0.05.pcd", *cloud);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    std::cout<<"loaded pdc"<<std::endl;
    std::cout << "with point num =  " << cloud->size() << " \n";
    auto start = std::chrono::high_resolution_clock::now();

    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;

    // Set the model you wish to fit
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);

    // Max distance for a point to be considered fitting the model
    // Adjust this parameter to get the appropriate amount of inliers
    seg.setDistanceThreshold(0.2);

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;

    // Create the point clouds for the inliers and outliers
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr inliers (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr outliers (new pcl::PointCloud<pcl::PointXYZRGB>);

    // Create the indices for the point cloud
    pcl::PointIndices::Ptr indices (new pcl::PointIndices);

    // Segment the point cloud
    seg.setInputCloud(cloud);
    seg.segment(*indices, *coefficients);

    // Extract the inliers
    extract.setInputCloud(cloud);
    extract.setIndices(indices);
    extract.setNegative(false);
    extract.filter(*inliers);


    std::chrono::duration<double> used = std::chrono::high_resolution_clock::now() - start;
    std::cout << "ransac segmenation used time: " << used.count() << "s,  ";
    // Save the inliers to a point cloud file
    pcl::io::savePCDFileASCII ("inliers.pcd", *inliers);

    pcl::visualization::CloudViewer viewer("Cloud Viewer");
    viewer.showCloud(inliers);
    while (!viewer.wasStopped())
    {
    }



    // Extract the outliers


    extract.setNegative(true);
    extract.filter(*outliers);

    // Save the outliers to a point cloud file
    pcl::io::savePCDFileASCII ("outliers.pcd", *outliers);
    pcl::visualization::CloudViewer viewer_("outlier");
    viewer_.showCloud(outliers);
    while (!viewer_.wasStopped())
    {

    }
}