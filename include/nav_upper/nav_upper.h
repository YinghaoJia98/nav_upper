#ifndef NAV_UPPER_H_
#define NAV_UPPER_H_
#include <ros/ros.h>
#include <string>
#include <nav_msgs/OccupancyGrid.h>

class NavManager
{
public:
    NavManager(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
    void initialize();
    void setupTimer();

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    nav_msgs::OccupancyGrid LocalCostMap_grid;

    ros::Subscriber CostMapSubscriber_;
    void LocalCostMapCallBack(const nav_msgs::OccupancyGrid &msg);

    ros::Timer UpdateTargetTimer;
    void UpdateTargetTimerCallBack(const ros::TimerEvent &event);

    std::mutex UpdateCostMapMutex_;
};

#endif
