#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>

int main(int argc, char** argv) {
    // Define Point Cloud types
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

    // Load source and target clouds
    PointCloud::Ptr cloud_source(new PointCloud);
    PointCloud::Ptr cloud_target(new PointCloud);

    if (pcl::io::loadPCDFile<pcl::PointXYZ>("output1.pcd", *cloud_source) == -1) {
        PCL_ERROR("Couldn't read source file\n");
        return -1;
    }
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("output2.pcd", *cloud_target) == -1) {
        PCL_ERROR("Couldn't read target file\n");
        return -1;
    }

    // ICP object
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud_source);
    icp.setInputTarget(cloud_target);
    icp.setMaximumIterations(200);   
    icp.setTransformationEpsilon(1e-9);   
    icp.setEuclideanFitnessEpsilon(1e-6); 

    // Output point cloud
    PointCloud::Ptr cloud_aligned(new PointCloud);
    icp.align(*cloud_aligned);

    if (icp.hasConverged()) {
        std::cout << "ICP converged." << std::endl
                  << "Score: " << icp.getFitnessScore() << std::endl;
        std::cout << "Transformation Matrix:" << std::endl;
        std::cout << icp.getFinalTransformation() << std::endl;

        pcl::io::savePCDFileASCII("aligned.pcd", *cloud_aligned);
    } else {
        std::cout << "ICP did not converge." << std::endl;
    }

    return 0;
}
