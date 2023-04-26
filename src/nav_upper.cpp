#include <nav_upper/nav_upper.h>
NavManager::NavManager(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private)
{
    // ROS_INFO("Hello, world.");
    initialize();
    setupTimer();
}

void NavManager::initialize()
{
    std::string CostMap_topic_;
    nh_.param<std::string>("/nav_upper/nav_manager_settings/CostMap_topic", CostMap_topic_,
                           std::string("/hello"));
    // std::cout << "The topic is " << CostMap_topic_ << std::endl;
    CostMapSubscriber_ = nh_.subscribe(CostMap_topic_.c_str(), 1,
                                       &NavManager::LocalCostMapCallBack, this);
}

void NavManager::setupTimer()
{
    if (1)
    {
        double UpdateTargetFps_;
        nh_.param<double>("/nav_upper/nav_manager_settings/UpdateTarget_Fps", UpdateTargetFps_, 1.0);
        double duration_UpdateTarget = 1.0 / (UpdateTargetFps_ + 0.00001);
        UpdateTargetTimer = nh_.createTimer(ros::Duration(duration_UpdateTarget),
                                            &NavManager::UpdateTargetTimerCallBack, this);
        // ROS_INFO("UpdateTargetFps_ is %f", UpdateTargetFps_);
    }
}

void NavManager::LocalCostMapCallBack(const nav_msgs::OccupancyGrid &msg)
{
    std::lock_guard<std::mutex> lock(UpdateCostMapMutex_);
    LocalCostMap_grid = msg;
}

void NavManager::UpdateTargetTimerCallBack(const ros::TimerEvent &event)
{
    std::lock_guard<std::mutex> lock(UpdateCostMapMutex_);
    ROS_INFO("hello");
    // TODO Judge if there is obstacle
    // TODO Judge if the obstacle could be avoid
    // TODO Select the new target
}