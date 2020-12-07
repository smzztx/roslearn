#include <iostream>
#include <pcl/registration/icp.h>
#include <pcl/point_types.h>

int main()
{
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    // Set the input source and target
    // icp.setInputCloud (cloud_source);
    // icp.setInputTarget (cloud_target);
    
    // // Set the max correspondence distance to 5cm (e.g., correspondences with higher
    // distances will be ignored) icp.setMaxCorrespondenceDistance (0.05);
    // // Set the maximum number of iterations (criterion 1)
    // icp.setMaximumIterations (50);
    // // Set the transformation epsilon (criterion 2)
    // icp.setTransformationEpsilon (1e-8);
    // // Set the euclidean distance difference epsilon (criterion 3)
    // icp.setEuclideanFitnessEpsilon (1);
    
    // // Perform the alignment
    // icp.align (cloud_source_registered);
    
    // // Obtain the transformation that aligned cloud_source to cloud_source_registered
    // Eigen::Matrix4f transformation = icp.getFinalTransformation ();

    std::cout << "lalala" << std::endl;
    return 0;
}