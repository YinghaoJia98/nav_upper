#ifndef NAV_UPPER_H_
#define NAV_UPPER_H_
#include <ros/ros.h>
#include <string>
#include <nav_msgs/OccupancyGrid.h>
#include <eigen3/Eigen/Dense>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

class NavManager
{
public:
    NavManager(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
    void initialize();
    void setupTimer();

    bool IfObstaclesExist(nav_msgs::OccupancyGrid CostMap,
                          Eigen::Vector3d robot_location,
                          double robot_width,
                          double x_expand,
                          double angle);

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    std::string WorldFrame_;
    std::string BodyFrame_;
    // std::string CostMapFrame_;

    double TimeResidualMax_;
    double robot_width_;

    nav_msgs::OccupancyGrid LocalCostMap_grid;

    ros::Subscriber CostMapSubscriber_;
    void LocalCostMapCallBack(const nav_msgs::OccupancyGrid &msg);

    ros::Timer UpdateTargetTimer;
    void UpdateTargetTimerCallBack(const ros::TimerEvent &event);

    std::mutex UpdateCostMapMutex_;
};

#endif
