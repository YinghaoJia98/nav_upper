#ifndef NAV_UPPER_H_
#define NAV_UPPER_H_
#include <ros/ros.h>
#include <string>
#include <nav_msgs/OccupancyGrid.h>
#include <eigen3/Eigen/Dense>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseResult.h>

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
    std::string CmdTopic_;

    double TimeResidualMax_;
    double robot_width_;
    double LookAhead_X_;
    double LookAhead_Time_;

    bool IfTimeInsteadOfConstantXAhead_;
    bool IfPubCmd_;

    nav_msgs::OccupancyGrid LocalCostMap_grid;
    geometry_msgs::Twist CmdMsg_;

    ros::Subscriber CostMapSubscriber_;
    void LocalCostMapCallBack(const nav_msgs::OccupancyGrid &msg);

    ros::Subscriber NavStatusSubscriber_;
    void NavStatusCallBack(const move_base_msgs::MoveBaseResult &msg);

    ros::Subscriber CmdVelSubscriber_;
    void CmdVelCallBack(const geometry_msgs::Twist &msg);

    ros::Publisher CmdVelPub_;

    ros::Timer UpdateTargetTimer;
    void UpdateTargetTimerCallBack(const ros::TimerEvent &event);

    ros::Timer CmdPubTimer;
    void CmdPubTimerCallBack(const ros::TimerEvent &event);

    std::mutex UpdateCostMapMutex_;
    std::mutex UpdateCmdMsg_;
};

#endif
