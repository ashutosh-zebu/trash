#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/conversions.h>

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <math.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#include <functional>  // for std::bind and _1

using std::placeholders::_1;

class localOccupancyPublisher : public rclcpp::Node     
{
public:
  localOccupancyPublisher() : Node("local_occupancy_grid_server"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
  {
    this->declare_parameter<double>("local_planner_radius", 10.0);
    
    local_planner_radius = this->get_parameter("local_planner_radius").as_double();

    local_occupancy_grid_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("/local_occupancy_grid", 10);
    pcl_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/local_points", 10);
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/points", 10, std::bind(&localOccupancyPublisher::local_voxel_filter, this, _1));

    
  }

private:
  void local_voxel_filter(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  { 
    try{
      float min_ = -(float)local_planner_radius;
      float max_ = (float)local_planner_radius;

      geometry_msgs::msg::TransformStamped transformStamped;
      if (tf_buffer_.canTransform("map", "base_link", msg->header.stamp, tf2::durationFromSec(0.5))) {
          transformStamped = tf_buffer_.lookupTransform("map", "base_link", msg->header.stamp);
      } else {
          RCLCPP_WARN(this->get_logger(), "Transform not available yet");
          return;
      }

      auto q = transformStamped.transform.rotation;

      // RCLCPP_INFO(this->get_logger(), 
      //           "Transform: Translation (%.2f, %.2f, %.2f)", 
      //           transformStamped.transform.translation.x,
      //           transformStamped.transform.translation.y,
      //           transformStamped.transform.translation.z,
      //           transformStamped.transform.translation.z
      //           // transformStamped.transform.rotation.x,
      //           // transformStamped.transform.rotation.y,
      //           // transformStamped.transform.rotation.z,

      //           );
      

      // Declare vectors and messages
      std::vector<std::array<float, 3>> occupied_voxels;
      visualization_msgs::msg::MarkerArray marker_array;

      // Prepare PCL containers
      pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
      pcl::PCLPointCloud2::Ptr voxel_cloud(new pcl::PCLPointCloud2());

      pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_x(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_y(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_z(new pcl::PointCloud<pcl::PointXYZ>());

      // Convert from ROS message to PCL
      pcl::fromROSMsg(*msg, *input_cloud);
      
      // std::cout<<input_cloud<<std::endl;

      // Apply passthrough filters
      pcl::PassThrough<pcl::PointXYZ> pass_x;
      pass_x.setInputCloud(input_cloud);
      pass_x.setFilterFieldName("x");
      pass_x.setFilterLimits(min_, max_);
      pass_x.filter(*cloud_filtered_x);

      pcl::PassThrough<pcl::PointXYZ> pass_y;
      pass_y.setInputCloud(cloud_filtered_x);
      pass_y.setFilterFieldName("y");
      pass_y.setFilterLimits(min_, max_);
      pass_y.filter(*cloud_filtered_y);

      pcl::PassThrough<pcl::PointXYZ> pass_z;
      pass_z.setInputCloud(cloud_filtered_y);
      pass_z.setFilterFieldName("z");
      pass_z.setFilterLimits(min_, max_);
      pass_z.filter(*cloud_filtered_z);

      // Convert filtered cloud to PCLPointCloud2 for voxel filtering
      pcl::toPCLPointCloud2(*cloud_filtered_z, *cloud);

      // Apply voxel filter
      pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_filter;
      voxel_filter.setInputCloud(cloud);
      voxel_filter.setLeafSize(0.2f, 0.2f, 0.2f);
      voxel_filter.filter(*voxel_cloud);

      // Convert back to PointXYZ
      pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::fromPCLPointCloud2(*voxel_cloud, *xyz_cloud);

      // Build octomap from voxelized cloud
      octomap::Pointcloud octo_cloud;
      for (const auto& pt : xyz_cloud->points) {
          if (!std::isnan(pt.x) && !std::isnan(pt.y) && !std::isnan(pt.z)) {
              octo_cloud.push_back(pt.x, pt.y, pt.z);
          }
      }

      double resolution = 0.2;
      octomap::OcTree tree(resolution);
      octomap::point3d sensor_origin(0.0f, 0.0f, 0.0f);
      // octomap::point3d sensor_origin(
      //                               transformStamped.transform.translation.x,
      //                               transformStamped.transform.translation.y,
      //                               transformStamped.transform.translation.z);

      tree.insertPointCloud(octo_cloud, sensor_origin);

      // auto qua = transformStamped.transform.rotation;

      // // Convert quaternion to RPY
      // tf2::Quaternion quat(qua.x, qua.y, qua.z, qua.w);
      // double roll, pitch, yaw;
      // tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);


      // for (octomap::OcTree::leaf_iterator it = tree.begin_leafs(), end = tree.end_leafs(); it != end; ++it) {
      //     if (tree.isNodeOccupied(*it)) {
      //         float x = it.getX() + transformStamped.transform.translation.x ;
      //         float y = it.getY() + transformStamped.transform.translation.y ;
      //         float z = it.getZ() + transformStamped.transform.translation.z ;
      //         occupied_voxels.push_back({x, y, z});
      //     }
      // }


      auto qua = transformStamped.transform.rotation;

      tf2::Quaternion quat(qua.x, qua.y, qua.z, qua.w);

      tf2::Vector3 translation(
          transformStamped.transform.translation.x,
          transformStamped.transform.translation.y,
          transformStamped.transform.translation.z
      );
      tf2::Transform transform;
      transform.setOrigin(translation);
      transform.setRotation(quat);

      for (octomap::OcTree::leaf_iterator it = tree.begin_leafs(), end = tree.end_leafs(); it != end; ++it) {
          if (tree.isNodeOccupied(*it)) {
              // Local position of voxel
              tf2::Vector3 point_local(it.getX(), it.getY(), it.getZ());

              // Apply rotation and translation
              tf2::Vector3 point_global = transform * point_local;

              occupied_voxels.push_back({(float)point_global.x(), (float)point_global.y(), (float)point_global.z()});
          }
        }

      // Build Marker for visualization
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "map";  // Change if needed
      marker.header.stamp = this->get_clock()->now();
      marker.ns = "local_occupancy";
      marker.id = 0;
      marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.scale.x = 0.2;
      marker.scale.y = 0.2;
      marker.scale.z = 0.2;
      marker.color.r = 1.0f;
      marker.color.g = 0.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0f;

      for (const auto& voxel : occupied_voxels) {
          geometry_msgs::msg::Point p;
          p.x = voxel[0];
          p.y = voxel[1];
          p.z = voxel[2];
          marker.points.push_back(p);
      }

      marker_array.markers.push_back(marker);

      // Publish data
      local_occupancy_grid_publisher->publish(marker_array);

      // Also publish filtered point cloud for debug/visualization
      sensor_msgs::msg::PointCloud2 output_msg;
      pcl::toROSMsg(*cloud_filtered_z, output_msg);
      output_msg.header = msg->header;
      pcl_pub->publish(output_msg);
  }
  catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
        }
}

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr local_occupancy_grid_publisher;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_pub;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  double local_planner_radius;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);                       
  rclcpp::spin(std::make_shared<localOccupancyPublisher>());  
  rclcpp::shutdown();                               
  return 0;
}
