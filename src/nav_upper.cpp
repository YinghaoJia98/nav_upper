#include <nav_upper/nav_upper.h>
NavManager::NavManager(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private)
{
    ROS_INFO("Hello, nav_upper.");
    initialize();
    setupTimer();
    IfPubCmd_ = false;
    IdSeqPub_ = 0;
    MoveBaseStatus_ = 0;
    NavManagerStatus_ = 0;
}

void NavManager::initialize()
{
    std::string CostMap_topic_, NavStatus_topic_, NavTarget_topic_;

    nh_.param<std::string>("/nav_upper/nav_manager_settings/CostMap_topic", CostMap_topic_,
                           std::string("/hello"));
    // std::cout << "The topic is " << CostMap_topic_ << std::endl;
    CostMapSubscriber_ = nh_.subscribe(CostMap_topic_.c_str(), 1,
                                       &NavManager::LocalCostMapCallBack, this);

    nh_.param<std::string>("/nav_upper/nav_manager_settings/NavStatus_topic", NavStatus_topic_,
                           std::string("/hello"));
    NavStatusSubscriber_ = nh_.subscribe(NavStatus_topic_.c_str(), 1,
                                         &NavManager::NavStatusCallBack, this);

    nh_.param<std::string>("/nav_upper/nav_manager_settings/WorldFrame", WorldFrame_,
                           std::string("map"));
    nh_.param<std::string>("/nav_upper/nav_manager_settings/BodyFrame", BodyFrame_,
                           std::string("base_link"));

    nh_.param<std::string>("/nav_upper/nav_manager_settings/CmdTopic", CmdTopic_,
                           std::string("/cmd_vel"));
    nh_.param<std::string>("/nav_upper/nav_manager_settings/NavTarget_topic", NavTarget_topic_,
                           std::string("/move_base_simple/goal"));
    CmdVelSubscriber_ = nh_.subscribe(CmdTopic_.c_str(), 1,
                                      &NavManager::CmdVelCallBack, this);
    CmdVelPub_ = nh_.advertise<geometry_msgs::Twist>(CmdTopic_.c_str(), 10);
    PlannerTargetPub_ = nh_.advertise<geometry_msgs::PoseStamped>(NavTarget_topic_.c_str(), 10);

    nh_.param<double>("/nav_upper/nav_manager_settings/RobotWidth", robot_width_, 0.5);
    nh_.param<double>("/nav_upper/nav_manager_settings/TimeResidualMax", TimeResidualMax_, 1.0);
    nh_.param<double>("/nav_upper/nav_manager_settings/LookAhead_X", LookAhead_X_, 1.0);
    nh_.param<double>("/nav_upper/nav_manager_settings/LookAhead_Time", LookAhead_Time_, 1.0);
    nh_.param<bool>("/nav_upper/nav_manager_settings/IfTimeInsteadOfConstantXAhead", IfTimeInsteadOfConstantXAhead_, true);

    NavManagerStart_server_ = nh_.advertiseService(
        "NavManager_Start", &NavManager::stdNavManagerStartCallback, this);
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
    if (1)
    {
        double CmdPub_Fps_;
        nh_.param<double>("/nav_upper/nav_manager_settings/CmdPub_Fps", CmdPub_Fps_, 10.0);
        double duration_CmdPub = 1.0 / (CmdPub_Fps_ + 0.00001);
        CmdPubTimer = nh_.createTimer(ros::Duration(duration_CmdPub),
                                      &NavManager::CmdPubTimerCallBack, this);
    }
    if (1)
    {
        double ModelSwitch_Fps_;
        nh_.param<double>("/nav_upper/nav_manager_settings/ModelSwitch_Fps", ModelSwitch_Fps_, 3.0);
        double duration_ModelSwitch = 1.0 / (ModelSwitch_Fps_ + 0.00001);
        ModelSwitchTimer = nh_.createTimer(ros::Duration(duration_ModelSwitch),
                                           &NavManager::ModelSwitchTimerCallBack, this);
    }
}

void NavManager::LocalCostMapCallBack(const nav_msgs::OccupancyGrid &msg)
{
    std::lock_guard<std::mutex> lock(UpdateCostMapMutex_);
    LocalCostMap_grid = msg;
}

void NavManager::CmdVelCallBack(const geometry_msgs::Twist &msg)
{
    std::lock_guard<std::mutex> lock2(UpdateCmdMsgReceivedMutex_);
    CmdMsg_ = msg;
}

void NavManager::NavStatusCallBack(const actionlib_msgs::GoalStatusArray &msg)
{
    std::lock_guard<std::mutex> lock3(UpdateMovePlannerStatusMutex_);
if(msg.status_list.size()>0)
{
    MoveBaseStatus_ = msg.status_list[msg.status_list.size() - 1].status;
}
}

void NavManager::UpdateTargetTimerCallBack(const ros::TimerEvent &event)
{
    std::lock_guard<std::mutex> lock(UpdateCostMapMutex_);
    std::lock_guard<std::mutex> lock2(UpdateCmdMsgReceivedMutex_);

    std::lock_guard<std::mutex> lock4(CmdPubMutex_);
    std::lock_guard<std::mutex> lock5(NavManagerMutex_);
    //ROS_INFO("hello");

    if (NavManagerStatus_ != 1)
    {
        return;
    }

    tf::TransformListener world_base_listener;
    tf::StampedTransform world_base_transform;
    try
    {
        world_base_listener.waitForTransform(WorldFrame_.c_str(), BodyFrame_.c_str(), ros::Time(0), ros::Duration(1.0));
        world_base_listener.lookupTransform(WorldFrame_.c_str(), BodyFrame_.c_str(),
                                            ros::Time(0), world_base_transform);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("There is something wrong when trying get robot pose to get target orientation.");
        ROS_ERROR("%s", ex.what());
        // ros::Duration(1.0).sleep();
    }
    tf::Quaternion q_robot_target = world_base_transform.getRotation();
    double yaw_robot_ = tf::getYaw(q_robot_target);
    Eigen::Vector3d RobotLocation_(world_base_transform.getOrigin().x(),
                                   world_base_transform.getOrigin().y(),
                                   yaw_robot_);
    bool IfObstaclesExist_;
    double x_expand_;
    if (IfTimeInsteadOfConstantXAhead_)
    {
        x_expand_ = CmdMsg_.linear.x * LookAhead_Time_;
    }
    else
    {
        x_expand_ = LookAhead_X_;
    }
    IfObstaclesExist_ = IfObstaclesExist(LocalCostMap_grid,
                                         RobotLocation_,
                                         robot_width_,
                                         x_expand_,
                                         0);
    if (!IfObstaclesExist_)
    {
        // IfPubCmd_ = true;
        return;
    }
    else
    {
        NavManagerStatus_ = 2;
        IfPubCmd_ = false;
        geometry_msgs::PoseStamped TargetPose_;
        TargetPose_.header.frame_id = WorldFrame_.c_str();
        TargetPose_.header.seq = IdSeqPub_;
        IdSeqPub_++;
        TargetPose_.header.stamp = ros::Time::now();
        TargetPose_.pose.position.x = RobotLocation_[0] + 2.0 * cos(yaw_robot_);
        TargetPose_.pose.position.y = RobotLocation_[1] + 2.0 * sin(yaw_robot_);
        TargetPose_.pose.position.z = world_base_transform.getOrigin().z();
        TargetPose_.pose.orientation.x = world_base_transform.getRotation().x();
        TargetPose_.pose.orientation.y = world_base_transform.getRotation().y();
        TargetPose_.pose.orientation.z = world_base_transform.getRotation().z();
        TargetPose_.pose.orientation.w = world_base_transform.getRotation().w();
        PlannerTargetPub_.publish(TargetPose_);
    }
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
        //ROS_ERROR("The time residual is so big that the operating state of NavUpper might be poor! The time residual is %f. The current time is %f and the map time is %f.", TimeResiduals_, CurrentTime_.toSec(), MapTime_.toSec());
        // return true;
    }

    double CostMapResolution = CostMap.info.resolution;
    int CostMapWidth = CostMap.info.width;
    std::string CostMapFrame_middle_ = CostMap.header.frame_id;
    if (CostMapFrame_middle_ != WorldFrame_)
    {
        ROS_ERROR("The frame of CostMap is not the World Frame.");
        // return true;
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

void NavManager::CmdPubTimerCallBack(const ros::TimerEvent &event)
{
    std::lock_guard<std::mutex> lock4(CmdPubMutex_);
    std::lock_guard<std::mutex> lock5(NavManagerMutex_);

    if (IfPubCmd_)
    {
        NavManagerStatus_ = 1;
        geometry_msgs::Twist CmdMsgPub_;
        CmdMsgPub_.angular.x = 0.0;
        CmdMsgPub_.angular.y = 0.0;
        CmdMsgPub_.angular.z = 0.0;
        CmdMsgPub_.linear.x = 0.5;
        CmdMsgPub_.linear.y = 0.0;
        CmdMsgPub_.linear.z = 0.0;
        CmdVelPub_.publish(CmdMsgPub_);
    }
}

void NavManager::ModelSwitchTimerCallBack(const ros::TimerEvent &event)
{
    std::lock_guard<std::mutex> lock3(UpdateMovePlannerStatusMutex_);
    std::lock_guard<std::mutex> lock4(CmdPubMutex_);
    std::lock_guard<std::mutex> lock5(NavManagerMutex_);

    if (NavManagerStatus_ == 2)
    {
        if (MoveBaseStatus_ == 3)
        {
            NavManagerStatus_ = 1;
            IfPubCmd_ = true;
        }
    }
}

bool NavManager::stdNavManagerStartCallback(std_srvs::Trigger::Request &req,
                                            std_srvs::Trigger::Response &res)
{
    std::lock_guard<std::mutex> lock4(CmdPubMutex_);
    std::lock_guard<std::mutex> lock5(NavManagerMutex_);
    if (NavManagerStatus_ == 2)
    {
        ROS_INFO("Now the robot is runing by planner and can not be started.");
        res.success = false;
        return false;
    }
    if (NavManagerStatus_ == 1)
    {
        ROS_INFO("Now the robot is going straight.");
        res.success = false;
        return false;
    }
    else
    {
        NavManagerStatus_ = 1;
        IfPubCmd_ = true;
    }
    res.success = true;
    return true;
}
