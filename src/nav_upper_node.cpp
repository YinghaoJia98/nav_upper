#include <ros/ros.h>
#include <nav_upper/nav_upper.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "nav_upper_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    NavManager NavManager_(nh, nh_private);
    int SpinnerThread_;
    nh.param<int>("/nav_upper/nav_manager_settings/SpinnerThread", SpinnerThread_, 1);
    ros::AsyncSpinner spinner(SpinnerThread_); // Use n threads
    spinner.start();
    ros::waitForShutdown();
    ROS_INFO("The nav_upper_node is closing, see you.");
    return 0;
}