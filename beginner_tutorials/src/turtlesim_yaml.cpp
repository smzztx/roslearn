  /***************************************

 * Copyright: 2016-2018(c) ROS小课堂 www.corvin.cn

 * Filename: turtlesim_dynamic.cpp

 * Author: corvin

 * Descrition: 使用turtlesim来测试dynamic_reconfigure

 *   动态更新节点参数.

 * History:

 *   20180503: init this file.

 ****************************************/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
 
using namespace std;
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtlesim_dynamic_node");
    ros::NodeHandle handle;
 
    string cmd_topic = "/turtle1/cmd_vel";
    int loop_rate = 1;
    double linear_x = 1.0;
    double angular_z = 1.0;
    bool move_flag = true;
 
    ros::param::get("~cmd_topic", cmd_topic);
    ros::param::get("~loop_rate", loop_rate);
    ros::param::get("~linear_x", linear_x);
    ros::param::get("~angular_z", angular_z);
    ros::param::get("~move_flag", move_flag);
 
    ros::Publisher cmdVelPub = handle.advertise<geometry_msgs::Twist>(cmd_topic, 1);
    geometry_msgs::Twist speed;
 
    ros::Rate rate(loop_rate);
    while(ros::ok())
    {
        if(move_flag)
        {
            speed.linear.x  = linear_x;
            speed.angular.z = angular_z;
            cmdVelPub.publish(speed);
        }
        ros::spinOnce();
        rate.sleep();
    }
}