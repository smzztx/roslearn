#include <ros/ros.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <iostream>
#include <stdio.h>
#define _LINE_LENGTH 300
int main(int argc, char **argv)
{
    ros::init(argc, argv, "dual_serial_restart");
    ros::NodeHandle nh;
    // ros::Subscriber sub = nh.subscribe("amy_base_interface/stat", 1, callback);
    FILE *dual_serial_p=NULL;
    FILE *ps_kill_talker=NULL;
    char line[_LINE_LENGTH];

    while(nh.ok())
    {
        sleep(5);
        ROS_INFO("open talker");
        dual_serial_p = popen("rosrun beginner_tutorials talker", "r");
        // if (NULL != dual_serial_p)
        // {
        //     while (fgets(line, _LINE_LENGTH, dual_serial_p) != NULL)
        //     {
        //         printf("line=%s\n", line);
        //     }
        // }
        // else
        // {
        //     return 1;
        // }
        sleep(5);
        ROS_INFO("close talker");
        // pclose(dual_serial_p);
        ps_kill_talker = popen("kill $(ps aux | grep talker | grep -v grep | awk '{print $2}')","r");

        pclose(dual_serial_p);
        pclose(ps_kill_talker);
        ROS_INFO("lalala");
    }
    return 0;
}

