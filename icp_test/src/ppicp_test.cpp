#include <iostream>
#include <pcl/registration/icp.h>
#include <pcl/point_types.h>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan>

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

void get_measurement_error(const std::vector<sensor_msgs::LaserScan> &laser_data, std::ofstream &fout_, double start_angle_, double end_angle_)
{
    bool first_data_flag = true;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source, cloud_target;

    for(auto const &scan_in : laser_data)
    {
        if(first_data_flag)
        {
            // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source (new pcl::PointCloud<pcl::PointXYZ>());
            cloud_source->clear();
            pcl::PointXYZ newPoint;
            newPoint.z = 0.0;
            double newPointAngle;

            int beamNum = scan_in->ranges.size();
            for (int i = 0; i < beamNum; i++)
            {
                if(std::isnormal(scan_in->ranges[i]))
                {
                    newPointAngle = scan_in->angle_min + scan_in->angle_increment * i;
                    newPoint.x = scan_in->ranges[i] * cos(newPointAngle);
                    newPoint.y = scan_in->ranges[i] * sin(newPointAngle);
                    cloud_source->push_back(newPoint);
                }
            }
            cloud_target = cloud_source;
        }else
        {
            cloud_source = cloud_target;
            cloud_target->clear();
            pcl::PointXYZ newPoint;
            newPoint.z = 0.0;
            double newPointAngle;

            int beamNum = scan_in->ranges.size();
            for (int i = 0; i < beamNum; i++)
            {
                if(std::isnormal(scan_in->ranges[i]))
                {
                    newPointAngle = scan_in->angle_min + scan_in->angle_increment * i;
                    newPoint.x = scan_in->ranges[i] * cos(newPointAngle);
                    newPoint.y = scan_in->ranges[i] * sin(newPointAngle);
                    cloud_target->push_back(newPoint);
                }
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
        icp.setTransformationEpsilon (1e-8);
        // Set the euclidean distance difference epsilon (criterion 3)
        icp.setEuclideanFitnessEpsilon (1);
        
        // Perform the alignment
        icp.align (cloud_source_registered);
        
        // Obtain the transformation that aligned cloud_source to cloud_source_registered
        Eigen::Matrix4f transformation = icp.getFinalTransformation ();
        
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
    std::string raw_data_file = file_path + bagname + "raw_data.log";
    std::vector<sensor_msgs::LaserScan> laser_data(0);
    std::ofstream fout(raw_data_file);

    readDataFromBag(bagfile, laser_topic, laser_data);
    get_measurement_error(laser_data, fout, start_angle, end_angle);

    fout.close();

    return 0;
}