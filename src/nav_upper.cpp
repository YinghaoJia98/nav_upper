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

    nh_.param<std::string>("/nav_upper/nav_manager_settings/WorldFrame", WorldFrame_,
                           std::string("map"));
    nh_.param<std::string>("/nav_upper/nav_manager_settings/BodyFrame", BodyFrame_,
                           std::string("base_link"));

    nh_.param<double>("/nav_upper/nav_manager_settings/RobotWidth", robot_width_, 0.5);
    nh_.param<double>("/nav_upper/nav_manager_settings/TimeResidualMax", TimeResidualMax_, 1.0);
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

bool NavManager::IfObstaclesExist(nav_msgs::OccupancyGrid CostMap,
                                  Eigen::Vector3d robot_location,
                                  double robot_width,
                                  double x_expand,
                                  double angle)
{
    ros::Time MapTime_ = CostMap.header.stamp;
    ros::Time CurrentTime_ = ros::Time::now();
    double TimeResiduals_ = CurrentTime_.toSec() - MapTime_.toSec();
    if (abs(TimeResiduals_) > TimeResidualMax_)
    {
        ROS_ERROR("The time residual is so big that the operating state of NavUpper might be poor! The time residual is %f. The current time is %f and the map time is %f.",
                  TimeResiduals_, CurrentTime_.toSec(), MapTime_.toSec());
    }

    double CostMapResolution = CostMap.info.resolution;
    int CostMapWidth = CostMap.info.width;
    std::string CostMapFrame_middle_ = CostMap.header.frame_id;
    if (CostMapFrame_middle_ != WorldFrame_)
    {
        ROS_ERROR("The frame of CostMap is not the World Frame.");
    }
    geometry_msgs::Pose CostMapOriginInWorldFrame = CostMap.info.origin;
    tf::Quaternion q_CostMap_Middle_(CostMapOriginInWorldFrame.orientation.x,
                                     CostMapOriginInWorldFrame.orientation.y,
                                     CostMapOriginInWorldFrame.orientation.z,
                                     CostMapOriginInWorldFrame.orientation.w);

    double YawOfCostMapInWorldFrame_;
    YawOfCostMapInWorldFrame_ = tf::getYaw(q_CostMap_Middle_);

    double target_x_in_robot_frame_ = x_expand * cos(angle);
    double target_y_in_robot_frame_ = x_expand * sin(angle);
    double yaw_middle_ = robot_location[2];
    double target_x_in_world_frame_ = robot_location[0] +
                                      target_x_in_robot_frame_ * cos(yaw_middle_) -
                                      target_y_in_robot_frame_ * sin(yaw_middle_);
    double target_y_in_world_frame_ = robot_location[1] +
                                      target_x_in_robot_frame_ * sin(yaw_middle_) +
                                      target_y_in_robot_frame_ * cos(yaw_middle_);
    Eigen::Vector2d start_pos_0(robot_location[0], robot_location[1]);
    Eigen::Vector2d end_pos_0(target_x_in_world_frame_, target_y_in_world_frame_);
    int size_in_y = robot_width / CostMapResolution;
    int size_y_used = size_in_y / 2;
    int bad_point_count_ = 0;
    for (int i = -size_y_used; i < size_y_used + 1; i++)
    {
        double y_change_in_robot_frame_ = i * CostMapResolution;
        double x_start_in_body_frame_ = y_change_in_robot_frame_ * (-sin(angle));
        double y_start_in_body_frame = y_change_in_robot_frame_ * (cos(angle));
        double x_start_in_world_frame = robot_location[0] +
                                        x_start_in_body_frame_ * cos(yaw_middle_) -
                                        y_start_in_body_frame * sin(yaw_middle_);
        double y_start_in_world_frame = robot_location[1] +
                                        x_start_in_body_frame_ * sin(yaw_middle_) +
                                        y_start_in_body_frame * cos(yaw_middle_);

        Eigen::Vector2d start_middle_in_world_frame_(x_start_in_world_frame, y_start_in_world_frame);
        Eigen::Vector2d end_middle_in_world_frame_ = end_pos_0 - start_pos_0 + start_middle_in_world_frame_;
        // Eigen::Vector2d direction = end_middle - start_middle;
        // double direction_norm = direction.norm();
        int x_expand_count = x_expand / CostMapResolution;
        for (int j = 0; j < x_expand_count; j++)
        {
            double x_middle_World = start_middle_in_world_frame_[0] + j * CostMapResolution * cos(yaw_middle_ + angle);
            double y_middle_World = start_middle_in_world_frame_[1] + j * CostMapResolution * sin(yaw_middle_ + angle);
            double x_middle_CostMap_middle = x_middle_World - CostMapOriginInWorldFrame.position.x;
            double y_middle_CostMap_middle = y_middle_World - CostMapOriginInWorldFrame.position.y;
            double x_middle_CostMap = x_middle_CostMap_middle * cos(YawOfCostMapInWorldFrame_) + y_middle_CostMap_middle * sin(YawOfCostMapInWorldFrame_);
            double y_middle_CostMap = -x_middle_CostMap_middle * sin(YawOfCostMapInWorldFrame_) + y_middle_CostMap_middle * cos(YawOfCostMapInWorldFrame_);
            int index_x_middle = x_middle_CostMap / CostMapResolution;
            int index_y_middle = y_middle_CostMap / CostMapResolution;
            int index_middle = index_x_middle + index_y_middle * CostMapWidth;
            double cost_middle = CostMap.data[index_middle];
            if (cost_middle > 50)
            {
                bad_point_count_++;
            }
        }
    }
    if (bad_point_count_ > 1)
    {
        return true;
    }
    else
    {
        return false;
    }
    return true;
}