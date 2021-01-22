#include <iostream>
#include <pcl/registration/icp.h>
#include <pcl/point_types.h>
#include <fstream>
#include <string>
#include <csm/csm_all.h>

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

void laser_odom(const std::vector<sensor_msgs::LaserScan> &laser_data, std::ofstream &fout_)
{
    bool first_data_flag = true;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source(new pcl::PointCloud<pcl::PointXYZ>()), cloud_target(new pcl::PointCloud<pcl::PointXYZ>());
    LDP ref_scan, sens_scan;
    for(auto const &scan_in : laser_data)
    {
        fout_ << scan_in.header.stamp << std::endl;
        /**
        {    //ScanToPCLPointCloud
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
        **/
        // LDP ldp;
        {   //ScanToLDP
            std::swap(ref_scan, sens_scan);
            unsigned int n = scan_in.ranges.size();
            sens_scan = ld_alloc_new(n);

            for (unsigned int i = 0; i < n; i++)
            {
                // calculate position in laser frame

                double r = scan_in.ranges[i];

                if (r > scan_in.range_min && r < scan_in.range_max)
                {
                // fill in laser scan data

                sens_scan->valid[i] = 1;
                sens_scan->readings[i] = r;
                }
                else
                {
                sens_scan->valid[i] = 0;
                sens_scan->readings[i] = -1;  // for invalid range
                }

                sens_scan->theta[i]    = scan_in.angle_min + i * scan_in.angle_increment;

                sens_scan->cluster[i]  = -1;
            }

            sens_scan->min_theta = sens_scan->theta[0];
            sens_scan->max_theta = sens_scan->theta[n-1];

            sens_scan->odometry[0] = 0.0;
            sens_scan->odometry[1] = 0.0;
            sens_scan->odometry[2] = 0.0;

            sens_scan->true_pose[0] = 0.0;
            sens_scan->true_pose[1] = 0.0;
            sens_scan->true_pose[2] = 0.0;
            if(first_data_flag)
            {
                first_data_flag = false;
                continue;
            }
        }

        int invalid_matches = 0;
        {   //CSM plicp
            sm_params params;
            params.max_angular_correction_deg = 10.0;
            params.max_linear_correction = 0.05;
            params.max_iterations = 1000;
            params.epsilon_xy = 0.000001;
            params.epsilon_theta = 0.000001;
            params.max_correspondence_dist = 0.3;
            params.sigma = 0.010;
            params.use_corr_tricks = 1;
            params.restart = 0;
            params.restart_threshold_mean_error = 0.01;
            params.restart_dt = 1.0;
            params.restart_dtheta = 0.1;
            params.clustering_threshold = 0.25;
            params.orientation_neighbourhood = 20;
            params.use_point_to_line_distance = 1;
            params.do_alpha_test = 0;
            params.do_alpha_test_thresholdDeg = 20.0;
            params.outliers_maxPerc = 0.90;
            params.outliers_adaptive_order = 0.7;
            params.outliers_adaptive_mult = 2.0;
            params.do_visibility_test = 0;
            params.do_compute_covariance = 0;
            params.debug_verify_tricks = 0;
            params.use_ml_weights = 0;
            params.use_sigma_weights = 0;
            params.laser[0] = 0;
            params.laser[1] = 0;
            params.laser[2] = 0;
            params.first_guess[0] = 0;
            params.first_guess[1] = 0;
            params.first_guess[2] = 0;
            params.min_reading = 0;
            params.max_reading = 30.0;
            sm_result results;
            results.cov_x_m = 0;
            results.dx_dy1_m = 0;
            results.dx_dy2_m = 0;

            params.laser_ref = ref_scan;
            params.laser_sens = sens_scan;
            sm_icp(&params, &results);
            
            // Obtain the transformation that aligned cloud_source to cloud_source_registered
            Eigen::Matrix4f transformation;
            transformation << cos(results.x[2]), -sin(results.x[2]), 0, results.x[0], sin(results.x[2]), cos(results.x[2]), 0, results.x[1], 0, 0, 1, 0, 0, 0, 0, 1;
            fout_ << "transformation: \n" << transformation << std::endl;
            fout_ << std::endl;
            static Eigen::Matrix4f final_laser_odom = Eigen::Matrix4f::Identity();
            final_laser_odom = transformation * final_laser_odom;
            fout_ << "final_laser_odom: \n" << final_laser_odom << std::endl;
            fout_ << std::endl;
            if(results.valid == 1){
                
            }else{
                std::cerr << "plicp error!!!" << std::endl;
                invalid_matches ++;
            }

            /* Transformation Composition
            *
            *  _        _      _       _     _                                         _
            * |   a_x   |     |   b_x   |   | a_x + b_x.cos(a_theta) - b_y.sin(a_theta) |
            * |   a_y   | [+] |   b_y   | = | a_y + b+x.sin(a_theta) + b_y.cos(a_theta) |
            * | a_theta |     | b_theta |   |           a_theta + b_theta               |
            *  -       -       -       -     -                                         -
            *
            */
/**
            last_odom.x = results.x[0] + scan_odom.back().x * cos(results.x[2]) - scan_odom.back().y * sin(results.x[2]);
            last_odom.y = results.x[1] + scan_odom.back().x * sin(results.x[2]) + scan_odom.back().y * cos(results.x[2]);
            last_odom.theta = rad_fix(results.x[2], scan_odom.back().theta);
            last_odom.nvalid = results.nvalid;
            last_odom.iterations = results.iterations;
            last_odom.error = results.error;**/
        }
/**
        {   //PCL icp
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
**/
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
    laser_odom(laser_data, fout);

    fout.close();

    return 0;
}