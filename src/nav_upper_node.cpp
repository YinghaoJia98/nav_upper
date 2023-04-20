#include <ros/ros.h>
#include <nav_upper/nav_upper.h>
int main(int argc, char **argv)
{
    ros::init(argc, argv, "nav_upper_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    NavManager NavManager_(nh, nh_private);
    ROS_INFO("The nav_upper_node is closing, see you.");
    return 0;
}