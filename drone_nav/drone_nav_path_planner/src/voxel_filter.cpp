#include <iostream>
#include <string>
#include <chrono>
#include <functional>
#include <memory>


#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"


// PCL includes
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>

// OctoMap includes
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#include <vector>
#include <array>

using namespace std::chrono_literals;

class GlobalOccupancyPublisher: public rclcpp::Node
{
    public:
      GlobalOccupancyPublisher(): Node("globaloccupancypublishernode"){
        pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
        pcl::PCLPointCloud2::Ptr voxel_cloud(new pcl::PCLPointCloud2());
        pcl::PCDReader cloud_reader;
        pcl::PCDWriter cloud_writer;

        std::string path = "/home/ashutosh/zebu_ws/src/drone_nav/drone_nav_path_planner/pcd/";

        // 1. Load the input point cloud
        if (cloud_reader.read(path + "cloudGlobal.pcd", *cloud) == -1) {
            std::cerr << "Error reading PCD file." << std::endl;
        }
        std::cout << "Source Cloud Points: " << cloud->width * cloud->height << std::endl;

        // 2. Apply voxel grid filter
        pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_filter;
        voxel_filter.setInputCloud(cloud);
        voxel_filter.setLeafSize(0.2f, 0.2f, 0.2f);  // Adjust voxel size here
        voxel_filter.filter(*voxel_cloud);

        std::cout << "Voxel Cloud Points: " << voxel_cloud->width * voxel_cloud->height << std::endl;

        // 3. Save voxelized point cloud
        cloud_writer.write(path + "voxelized.pcd", *voxel_cloud, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), false);

        // 4. Convert to PointXYZ
        pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromPCLPointCloud2(*voxel_cloud, *xyz_cloud);

        // this change is wrt simulation pcl shift of one block
        float z_shift = 0.2f;

        // 5. Convert to octomap::Pointcloud
        octomap::Pointcloud octo_cloud;
        for (const auto& pt : xyz_cloud->points) {
            if (!std::isnan(pt.x) && !std::isnan(pt.y) && !std::isnan(pt.z)) {
                octo_cloud.push_back(pt.x, pt.y, pt.z + z_shift);
            }
        }

        // 6. Create an octree and insert the cloud
        double resolution = 0.2;  // m resolution
        octomap::OcTree tree(resolution);
        octomap::point3d sensor_origin(0.0f, 0.0f, 0.0f);  

        tree.insertPointCloud(octo_cloud, sensor_origin);

        
        // 7. Save OctoMap to binary file
        std::string octo_filename = path + "cloud_map.bt";
        tree.writeBinary(octo_filename);
        std::cout << "OctoMap written to: " << octo_filename << std::endl;
    
        

        for (octomap::OcTree::leaf_iterator it = tree.begin_leafs(),
                                            end = tree.end_leafs();
            it != end; ++it) {
            if (tree.isNodeOccupied(*it)) {
                float x = it.getX();
                float y = it.getY();
                float z = it.getZ();
                occupied_voxels.push_back({x, y, z});
            }
        }
        
        // Example: print number of occupied voxels
        std::cout << "Number of occupied voxels: " << occupied_voxels.size() << std::endl;

        // for (size_t i = 0; i < occupied_voxels.size(); i++) {
        // const auto& voxel = occupied_voxels[i];
        // std::cout << "Voxel " << i << ": (" 
        //         << voxel[0] << ", " << voxel[1] << ", " << voxel[2] << ")\n";
        // }

        publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/global_occupancy_grid", 10);
        timer_ = this->create_wall_timer(60ms, std::bind(&GlobalOccupancyPublisher::timer_callback, this));
        // timer_callback();

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "global_occupancy";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;



        for (const auto &voxel : occupied_voxels) {
            // std::cout << "Voxel X: ";
            // std::cout << voxel[0] << std::endl;

            // std::cout << "Voxel Y: ";
            // std::cout << voxel[1] << std::endl;

            // std::cout << "Voxel Z: ";
            // std::cout << voxel[2] << std::endl;
            geometry_msgs::msg::Point p;
            p.x = voxel[0];
            p.y = voxel[1];
            p.z = voxel[2];
            marker.points.push_back(p);
        }

        
        marker_array.markers.push_back(marker);
      }

      private:
      std::vector<std::array<float, 3>> occupied_voxels;
      visualization_msgs::msg::MarkerArray marker_array;
      void timer_callback(){

            publisher_->publish(marker_array);
            // std::cout << "Hello" << std::endl;
        
    }
    const double octree_resolution_ = 0.2;
    // std::vector<std::array<float, 3>> occupied_voxels_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GlobalOccupancyPublisher>());
  rclcpp::shutdown();
  return 0;
}