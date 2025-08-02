#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#include <Eigen/Dense>
#include <functional>

using std::placeholders::_1;

class GlobalOccupancyPublisher : public rclcpp::Node
{
public:
  GlobalOccupancyPublisher() 
    : Node("global_occupancy_grid_server"), 
      tf_buffer_(this->get_clock()), 
      tf_listener_(tf_buffer_)
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/lidar/points", 10, std::bind(&GlobalOccupancyPublisher::pointCloudCallback, this, _1));

    map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/global_point_cloud", 1);

    global_tree_ = std::make_shared<octomap::OcTree>(0.2);  // 20 cm resolution
  }

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    try {
      // Transform from base_link to map
      geometry_msgs::msg::TransformStamped transformStamped;
      if (tf_buffer_.canTransform("map", "base_link", msg->header.stamp, tf2::durationFromSec(0.5))) {
          transformStamped = tf_buffer_.lookupTransform("map", "base_link", msg->header.stamp);
      } else {
          RCLCPP_WARN(this->get_logger(), "Transform not available yet");
          return;
      }
      // Convert PointCloud2 to PCL
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromROSMsg(*msg, *cloud_in);

      // Convert transform to Eigen
      geometry_msgs::msg::Quaternion q = transformStamped.transform.rotation;
      geometry_msgs::msg::Vector3 t = transformStamped.transform.translation;

      Eigen::Quaternionf rot(q.w, q.x, q.y, q.z);
      Eigen::Vector3f trans(t.x, t.y, t.z);
      Eigen::Affine3f transform = Eigen::Translation3f(trans) * rot;


      // Transform point cloud to map frame
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_map(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::transformPointCloud(*cloud_in, *cloud_map, transform);

      // Optional: apply voxel grid filter to incoming cloud
      pcl::VoxelGrid<pcl::PointXYZ> voxel;
      voxel.setInputCloud(cloud_map);
      voxel.setLeafSize(0.2f, 0.2f, 0.2f);
      pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
      voxel.filter(*filtered);

      // Insert points into OctoMap
      for (const auto& pt : filtered->points) {
        global_tree_->updateNode(octomap::point3d(pt.x, pt.y, pt.z), true);
      }

      // Publish map as PointCloud2
      pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      for (auto it = global_tree_->begin_leafs(), end = global_tree_->end_leafs(); it != end; ++it) {
        if (global_tree_->isNodeOccupied(*it)) {
          map_cloud->points.emplace_back(it.getX(), it.getY(), it.getZ());
        }
      }

      sensor_msgs::msg::PointCloud2 output_msg;
      pcl::toROSMsg(*map_cloud, output_msg);
      output_msg.header.frame_id = "map";
      output_msg.header.stamp = this->get_clock()->now();
      map_pub_->publish(output_msg);

    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::shared_ptr<octomap::OcTree> global_tree_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GlobalOccupancyPublisher>());
  rclcpp::shutdown();
  return 0;
}
