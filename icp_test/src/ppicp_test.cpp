#include <iostream>
#include <pcl/registration/icp.h>
#include <pcl/point_types.h>
#include <fstream>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

void readDataFromBag(const std::string &bag_name, const std::string &laser_topic_name, std::vector<sensor_msgs::LaserScan> &laser_data)
{
    rosbag::Bag bag;
    std::cout << "Opening bag..." << std::endl;

    try{
        bag.open(bag_name, rosbag::bagmode::Read);
    }catch (std::exception& e) {
        std::cerr << "Opening bag ERROR!" << std::endl;
        throw;
    }

    std::cout << "Opening bag DONE!" << std::endl;
    std::cout << "Quering topics bag..." << std::endl;

    std::vector<std::string> topics;
    topics.push_back(laser_topic_name);

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    std::cout << "Reading bag data..." << std::endl;

    for (rosbag::MessageInstance const m: view) {
        sensor_msgs::LaserScanConstPtr scan = m.instantiate<sensor_msgs::LaserScan>();
        if (scan != nullptr) {
        sensor_msgs::LaserScan tmp(*scan);
        laser_data.push_back(tmp);
        }
    }

    std::cout << "laser size: " << laser_data.size() << std::endl;

    std::cout << "Data reading finished!" << std::endl;

    return;
}

void scan_match(const std::vector<sensor_msgs::LaserScan> &laser_data, std::ofstream &fout_)
{
    bool first_data_flag = true;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source(new pcl::PointCloud<pcl::PointXYZ>()), cloud_target(new pcl::PointCloud<pcl::PointXYZ>());

    for(auto const &scan_in : laser_data)
    {
        fout_ << scan_in.header.stamp << std::endl;
        {
            std::swap(cloud_source, cloud_target);
            cloud_target->clear();
            pcl::PointXYZ newPoint;
            newPoint.z = 0.0;
            double newPointAngle;

            int beamNum = scan_in.ranges.size();
            for (int i = 0; i < beamNum; i++)
            {
                if(std::isnormal(scan_in.ranges[i]))
                {
                    newPointAngle = scan_in.angle_min + scan_in.angle_increment * i;
                    newPoint.x = scan_in.ranges[i] * cos(newPointAngle);
                    newPoint.y = scan_in.ranges[i] * sin(newPointAngle);
                    cloud_target->push_back(newPoint);
                }
            }
            if(first_data_flag)
            {
                first_data_flag = false;
                continue;
            }
        }

        pcl::PointCloud<pcl::PointXYZ> cloud_source_registered;
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        // Set the input source and target
        icp.setInputSource (cloud_source);
        icp.setInputTarget (cloud_target);
        
        // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
        icp.setMaxCorrespondenceDistance (0.05);
        // Set the maximum number of iterations (criterion 1)
        icp.setMaximumIterations (50);
        // Set the transformation epsilon (criterion 2)
        // icp.setTransformationEpsilon (1e-8);
        // Set the euclidean distance difference epsilon (criterion 3)
        // icp.setEuclideanFitnessEpsilon (1);
        
        // Perform the alignment
        icp.align (cloud_source_registered);
        fout_ << "cloud_source_registered: \n" << cloud_source_registered << std::endl;
        
        // Obtain the transformation that aligned cloud_source to cloud_source_registered
        Eigen::Matrix4f transformation = icp.getFinalTransformation ();
        fout_ << "transformation: \n" << transformation << std::endl;
        fout_ << std::endl;
        static Eigen::Matrix4f final_laser_odom = Eigen::Matrix4f::Identity();
        final_laser_odom = transformation * final_laser_odom;
        fout_ << "final_laser_odom: \n" << final_laser_odom << std::endl;
        fout_ << std::endl;
        if (icp.hasConverged ())
        {
            fout_ << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
        }
        else
        {
            PCL_ERROR ("\nICP has not converged.\n");
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ppicp_test");
    ros::NodeHandle n("~");

    std::string file_path;
    std::string bagname;
    std::string laser_topic;

    ros::param::get("~file_path", file_path);
    ros::param::get("~bagname", bagname);
    ros::param::get("~laser_topic", laser_topic);

    std::string bagfile = file_path + bagname + ".bag";
    std::string laser_odom_file = file_path + bagname + "laser_odom.log";
    std::vector<sensor_msgs::LaserScan> laser_data(0);
    std::ofstream fout(laser_odom_file);

    // readDataFromBag(bagfile, laser_topic, laser_data);
    readDataFromBag("/home/txcom-ubuntu64/2020-12-10-11-21-37.bag", "/scan", laser_data);
    scan_match(laser_data, fout);

    fout.close();

    return 0;
}