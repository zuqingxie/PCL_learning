# include "octomap_projection.h"

class octomap_project:{
public:

private:


}





namespace octomap_project

{
  OctomapProject::OctomapProject(): Node("octomap_projection")
  {
    InitParams();
  }
  
  OctomapProject::~OctomapProject()
  {

  }


  void OctomapProject::OctomapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_PassThrough(new pcl::PointCloud<pcl::PointXYZ>);
  }

  void OctomapProject::EachGridmap()
  {
   PassThroughFilter(false);
   SetMapTopicMsg(cloud_after_PassThrough_, map_topic_msg_);
  }

  void OctomapProject::PassThroughFilter(const bool& flag_in)
  {
    // 初始化,并通过tf2_ros::TransformListener完成对tf2_ros::Buffer类的初始化和构造，并订阅相应tf消息
    buffers_.reset(new tf2_ros::Buffer(this->get_clock()));
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*buffers_.get());


    /*方法一：直通滤波器对点云进行处理。*/
    cloud_after_PassThrough_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PassThrough<pcl::PointXYZ> passthrough;
    passthrough.setInputCloud(pcd_cloud_);//输入点云
    passthrough.setFilterFieldName("x");//对x轴进行操作
    passthrough.setFilterLimits(thre_x_min_, thre_x_max_);//设置直通滤波器操作范围
    passthrough.setFilterLimitsNegative(flag_in);//true表示保留范围外，false表示保留范围内
    passthrough.filter(*cloud_after_PassThrough_);//执行滤波，过滤结果保存在 cloud_after_PassThrough_

    passthrough.setFilterFieldName("y");//对y轴进行操作
    passthrough.setFilterLimits(thre_y_min_, thre_y_max_);//设置直通滤波器操作范围
    passthrough.filter(*cloud_after_PassThrough_);

    passthrough.setFilterFieldName("z");//对z轴进行操作
    passthrough.setFilterLimits(thre_z_min_, thre_z_max_);//设置直通滤波器操作范围
    passthrough.filter(*cloud_after_PassThrough_);

    std::cout << "直通滤波后点云数据点数：" << cloud_after_PassThrough_->points.size() << std::endl;
    try
    {
      echo_transform = buffers_->lookupTransform(world_frame_id_,robot_frame_id_,tf2::TimePoint());
      Eigen::Matrix4f matrix_transform = pcl_ros::transformAsMatrix(echo_transform);

      pcl::transformPointCloud(*cloud_after_PassThrough_.get(),*cloud_after_PassThrough_.get(),matrix_transform);
    }
    catch(const tf2::TransformException& ex)
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Transform error of sensor data: " << ex.what() << ", quitting callback");
    }

  }

  void OctomapProject::SetMapTopicMsg(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, nav_msgs::msg::OccupancyGrid& msg)
  {
    msg.header.stamp = builtin_interfaces::msg::Time(this->now());;
    msg.header.frame_id = "map";
    msg.info.map_load_time = builtin_interfaces::msg::Time(this->now());;
    msg.info.resolution = map_resolution_;

    double x_min, x_max, y_min, y_max;
    double z_max_grey_rate = 0.05;
    double z_min_grey_rate = 0.95;
    double k_line = (z_max_grey_rate - z_min_grey_rate) / (thre_z_max_ - thre_z_min_);
    double b_line = (thre_z_max_ * z_min_grey_rate - thre_z_min_ * z_max_grey_rate) / (thre_z_max_ - thre_z_min_);

    if(cloud->points.empty())
    {
      RCLCPP_WARN(this->get_logger(),"pcd is empty!\n");
      return;
    }

    for(int i = 0; i < cloud->points.size() - 1; i++)
    {
      if(i == 0)
      {
        x_min = x_max = cloud->points[i].x;
        y_min = y_max = cloud->points[i].y;
      }

      double x = cloud->points[i].x;
      double y = cloud->points[i].y;

      if(x < x_min) x_min = x;
      if(x > x_max) x_max = x;

      if(y < y_min) y_min = y;
      if(y > y_max) y_max = y;
    }

    msg.info.origin.position.x = x_min;
    msg.info.origin.position.y = y_min;
    msg.info.origin.position.z = 0.0;
    msg.info.origin.orientation.x = 0.0;
    msg.info.origin.orientation.y = 0.0;
    msg.info.origin.orientation.z = 0.0;
    msg.info.origin.orientation.w = 1.0;

    msg.info.width = int((x_max - x_min) / map_resolution_);//可以根据x_max和x_min来设置地图动态大小
    msg.info.height = int((y_max - y_min) / map_resolution_);

    msg.data.resize(msg.info.width * msg.info.height);
    msg.data.assign(msg.info.width * msg.info.height, 0);

    RCLCPP_INFO(this->get_logger(), "data size = %d\n", msg.data.size());

    for(int iter = 0; iter < cloud->points.size(); iter++)
    {
      int i = int((cloud->points[iter].x - x_min) / map_resolution_);
      if(i < 0 || i >= msg.info.width) continue;

      int j = int((cloud->points[iter].y - y_min) / map_resolution_);
      if(j < 0 || j >= msg.info.height - 1) continue;
      msg.data[i + j * msg.info.width] = 100;//int(255 * (cloud->points[iter].z * k_line + b_line)) % 255;
    }
  }

  bool OctomapProject::InitParams()
  {
    world_frame_id_ = "map";
    robot_frame_id_ = "robot";
    this->declare_parameter<float>("thre_x_min", -0.0);
    this->get_parameter_or<float>("thre_x_min", thre_x_min_, 0.0);
    this->declare_parameter<float>("thre_x_max", 2.0);
    this->get_parameter_or<float>("thre_x_max", thre_x_max_, 2.0);

    this->declare_parameter<float>("thre_y_min", 0.0);
    this->get_parameter_or<float>("thre_y_min", thre_y_min_, 0.0);
    this->declare_parameter<float>("thre_y_max", 2.0);
    this->get_parameter_or<float>("thre_y_max", thre_y_max_, 2.0);

    this->declare_parameter<float>("thre_z_min", 0.0);
    this->get_parameter_or<float>("thre_z_min", thre_z_min_, 0.0);
    this->declare_parameter<float>("thre_z_max", 2.0);
    this->get_parameter_or<float>("thre_z_max", thre_z_max_, 2.0);

    this->declare_parameter<float>("map_resolution", 0.05);
    this->get_parameter_or<float>("map_resolution", map_resolution_, 0.05);

    gridmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("grid_map", 1);

    fullMapPub_ = this->create_publisher<octomap_msgs::msg::Octomap>("octomap_full", 1);

    std::string pcd_file = "src/octomap_server/dat/pointcloudmap_2661935000.pcd";
    pcd_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    if(pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file,*pcd_cloud_) == -1)
    {
     PCL_ERROR ("Couldn't read file: %s \n", pcd_file.c_str());
     return -1;
    }


    std::cout << "初始点云数据点数：" << pcd_cloud_->points.size() << std::endl;
    EachGridmap();

    rclcpp::WallRate  loop_rate(5000);
    while (rclcpp::ok())
    {
        gridmap_pub_->publish(map_topic_msg_);
        loop_rate.sleep();
    }
    return 0;
  }
}

int
main (int argc, char** argv)
{




}