#include "pid_position_follow.hpp"

namespace pid_position_follow
{
// 构造函数，初始化RobotCtrl类
RobotCtrl::RobotCtrl(const rclcpp::NodeOptions &options) : Node("pid_position_follow", options) {
    // 声明参数，并设置默认值
    this->declare_parameter("max_x_speed", 1.0); // 最大X方向速度
    this->declare_parameter("max_y_speed", 1.0); // 最大Y方向速度
    this->declare_parameter("set_yaw_speed", 0.0); // 设置的偏航速度
    this->declare_parameter("p_value", 0.5); // PID控制器的P值
    this->declare_parameter("i_value", 1.0); // PID控制器的I值
    this->declare_parameter("d_value", 1.0); // PID控制器的D值
    this->declare_parameter("plan_frequency", 30); // 规划频率
    this->declare_parameter("goal_dist_tolerance", 0.2); // 目标距离容差
    this->declare_parameter("prune_ahead_distance", 0.5); // 修剪前瞻距离
    this->declare_parameter("global_frame", "map"); // 全局坐标系名称

    // 获取参数值并赋值给成员变量
    max_x_speed_ = this->get_parameter("max_x_speed").get_value<double>();
    max_y_speed_ = this->get_parameter("max_y_speed").get_value<double>();
    set_yaw_speed_ = this->get_parameter("set_yaw_speed").get_value<double>();
    p_value_ = this->get_parameter("p_value").get_value<double>();
    i_value_ = this->get_parameter("i_value").get_value<double>();
    d_value_ = this->get_parameter("d_value").get_value<double>();
    plan_freq_ = this->get_parameter("plan_frequency").get_value<int>();
    goal_dist_tolerance_ = this->get_parameter("goal_dist_tolerance").get_value<double>();
    prune_ahead_dist_ = this->get_parameter("prune_ahead_distance").get_value<double>();
    global_frame_ = this->get_parameter("global_frame").get_value<std::string>();

    // 创建发布者，发布云台偏航位置命令
    gimbal_yaw_position_cmd_ = this->create_publisher<std_msgs::msg::Float64>("/auto_car/yaw_steering_position_controller/command", 10);
    // 创建发布者，发布本地路径
    local_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/p_path", 5);
    // 创建订阅者，订阅全局路径
    global_path_sub_ = this->create_subscription<nav_msgs::msg::Path>("/plan", 5, std::bind(&RobotCtrl::GlobalPathCallback, this, std::placeholders::_1));
    // 创建发布者，发布速度命令
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/base_vel", 10);
    //创建服务，处理PID规划状态请求
    planner_server_ = this->create_service<rm_decision_interfaces::srv::PidPlannerStatus>("/pid_planner_status", 
        [this](const std::shared_ptr<rm_decision_interfaces::srv::PidPlannerStatus::Request> req,
               std::shared_ptr<rm_decision_interfaces::srv::PidPlannerStatus::Response> resp) {
            return this->Follower_StateReq(req, resp);
        });

    // // 创建服务，处理决策请求
    // decision_server_ = 
    //     this->create_service<rm_decision_interfaces::srv::Decision>(
    //         "decision_server",
    //         [this](const std::shared_ptr<rm_decision_interfaces::srv::Decision::Request> request,
    //             std::shared_ptr<rm_decision_interfaces::srv::Decision::Response> response) {
    //             this->serverCallback(request, response);
    //         });

    // 创建TF缓冲区
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    // 创建TF监听器
    transform_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 设置定时器的周期为50毫秒
    auto period = std::chrono::milliseconds(50);
        
        // 创建一个定时器，绑定到timerCallback函数
    timer_ = this->create_wall_timer(period, [this]() { this->timerCallback(nullptr); });
}

// 服务回调函数，处理决策请求
// void RobotCtrl::serverCallback(
//     const std::shared_ptr<rm_decision_interfaces::srv::Decision::Request> request,
//     std::shared_ptr<rm_decision_interfaces::srv::Decision::Response> response) 
//     {
//         RCLCPP_INFO(this->get_logger(),"serverCallback");
//         // RCLCPP_INFO(this->get_logger(), "收到a: %ld b: %ld", request->a,
//         //             request->b);
//         // response->sum = request->a + request->b;
//     }

// 规划函数，定时调用
void RobotCtrl::timerCallback([[maybe_unused]] const rclcpp::TimerBase::SharedPtr event) {

    
        if (plan_) {
            auto begin = std::chrono::steady_clock::now();
            auto start = this->now();
            // 1. 更新从全局路径坐标系到本地规划坐标系的变换
            geometry_msgs::msg::TransformStamped transform_stamped;
            try {
                transform_stamped = tf_buffer_->lookupTransform(global_frame_, global_path_.header.frame_id, tf2::TimePointZero);
            } catch (tf2::TransformException &ex) {
                RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
                return;
            }

            // 2. 获取当前机器人在全局路径坐标系中的姿态
            geometry_msgs::msg::PoseStamped robot_pose;
            GetGlobalRobotPose(tf_buffer_, global_path_.header.frame_id, robot_pose);

            // 3. 检查机器人是否已经到达目标点，距离容差为goal_dist_tolerance_
            if (GetEuclideanDistance(robot_pose, global_path_.poses.back()) <= goal_dist_tolerance_ || prune_index_ == static_cast<int>(global_path_.poses.size() - 1)) {
                plan_ = false;
                geometry_msgs::msg::Twist cmd_vel;
                cmd_vel.linear.x = 0;
                cmd_vel.linear.y = 0;
                cmd_vel.angular.z = set_yaw_speed_;
                cmd_vel.linear.z = 1;   // bool success or not
                cmd_vel_pub_->publish(cmd_vel);
                RCLCPP_INFO(this->get_logger(), "Planning Success!");
                return;
            }

            // 4. 从给定的全局路径中获取修剪索引
            FindNearstPose(robot_pose, global_path_, prune_index_, prune_ahead_dist_);

            // 5. 生成修剪路径并将其转换到本地规划坐标系
            nav_msgs::msg::Path prune_path, local_path;

            local_path.header.frame_id = global_frame_;
            prune_path.header.frame_id = global_frame_;

            geometry_msgs::msg::PoseStamped tmp_pose;
            tmp_pose.header.frame_id = global_frame_;

            tf2::doTransform(robot_pose, tmp_pose, transform_stamped);
            prune_path.poses.push_back(tmp_pose);

            size_t i = prune_index_;

            while (i < global_path_.poses.size() && i - prune_index_ < 20) {
                tf2::doTransform(global_path_.poses[i], tmp_pose, transform_stamped);
                prune_path.poses.push_back(tmp_pose);
                i++;
            }

            // 6. 从上述修剪路径生成三次样条轨迹
            GenTraj(prune_path, local_path);
            local_path_pub_->publish(local_path);

            // 7. 跟随轨迹并计算速度
            geometry_msgs::msg::Twist cmd_vel;
            FollowTraj(robot_pose, local_path, cmd_vel);
            cmd_vel_pub_->publish(cmd_vel);
            GimbalCtrl();

            auto plan_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - begin);
            RCLCPP_INFO(this->get_logger(), "Planning takes %f ms and passed %d/%d.",
                        plan_time.count() / 1000.,
                        prune_index_,
                        static_cast<int>(global_path_.poses.size()));
        }
        else {
            geometry_msgs::msg::Twist cmd_vel;
            cmd_vel.linear.x = 0;
            cmd_vel.linear.y = 0;
            cmd_vel.angular.z = set_yaw_speed_;
            cmd_vel.linear.z = 0;   // bool success or not
            cmd_vel_pub_->publish(cmd_vel);
            GimbalCtrl();
        }
    
}

// 查找最近的姿态函数
void RobotCtrl::FindNearstPose(geometry_msgs::msg::PoseStamped& robot_pose, nav_msgs::msg::Path& path, int& prune_index, double prune_ahead_dist) {
    double dist_threshold = 10; // 阈值为10米
    double sq_dist_threshold = dist_threshold * dist_threshold; // 计算阈值的平方，用于比较距离
    double sq_dist; // 存储当前距离的平方

    // 如果prune_index不为0，计算当前prune_index前一个点的距离平方
    if (prune_index != 0) {
        sq_dist = GetEuclideanDistance(robot_pose, path.poses[prune_index - 1]);
    } else {
        sq_dist = 1e10; // 如果prune_index为0，设置一个非常大的距离平方
    }

    double new_sq_dist = 0; // 存储新的距离平方

    // 遍历路径中的所有点，直到找到最近的点
    while (prune_index < (int)path.poses.size()) {
        new_sq_dist = GetEuclideanDistance(robot_pose, path.poses[prune_index]); // 计算当前点的距离平方

        // 如果新的距离平方大于当前距离平方且当前距离平方小于阈值
        if (new_sq_dist > sq_dist && sq_dist < sq_dist_threshold) {
            // 判断是否在同一方向且sq_dist大于0.3米
            if ((path.poses[prune_index].pose.position.x - robot_pose.pose.position.x) *
                (path.poses[prune_index - 1].pose.position.x - robot_pose.pose.position.x) +
                (path.poses[prune_index].pose.position.y - robot_pose.pose.position.y) *
                (path.poses[prune_index - 1].pose.position.y - robot_pose.pose.position.y) > 0
                && sq_dist > prune_ahead_dist) {
                prune_index--; // 如果满足条件，回退一个点
            } else {
                sq_dist = new_sq_dist; // 否则更新当前距离平方
            }

            break; // 退出循环
        }

        sq_dist = new_sq_dist; // 更新当前距离平方
        ++prune_index; // 移动到下一个点
    }

    // 确保prune_index不超过路径中的最后一个点的索引
    prune_index = std::min(prune_index, (int)(path.poses.size() - 1));
}

// 跟随轨迹函数
void RobotCtrl::FollowTraj(const geometry_msgs::msg::PoseStamped& robot_pose, const nav_msgs::msg::Path& traj, geometry_msgs::msg::Twist& cmd_vel) {
    // 定义一个PoseStamped对象，用于存储机器人的全局姿态
    geometry_msgs::msg::PoseStamped robot_pose_1;
    // 调用GetGlobalRobotPose函数，获取机器人在全局坐标系中的姿态，并存储在robot_pose_1中
    GetGlobalRobotPose(tf_buffer_, global_path_.header.frame_id, robot_pose_1);

    // 使用tf2::getYaw函数从机器人的四元数中提取偏航角（yaw），并存储在yaw_变量中
    yaw_ = tf2::getYaw(robot_pose_1.pose.orientation);

    // 计算机器人当前位置与轨迹中第二个点之间的偏航角差值
    // 使用atan2函数计算两个点之间的角度差，参数为y和x的差值
    double diff_yaw = atan2((traj.poses[1].pose.position.y - robot_pose.pose.position.y), (traj.poses[1].pose.position.x - robot_pose.pose.position.x));

    // 计算机器人当前位置与轨迹中第二个点之间的欧几里得距离
    double diff_distance = GetEuclideanDistance(robot_pose, traj.poses[1]);

    // 将偏航角差值限制在-π到π之间
    if (diff_yaw > M_PI) {
        diff_yaw -= 2 * M_PI;
    } else if (diff_yaw < -M_PI) {
        diff_yaw += 2 * M_PI;
    }

    // 打印偏航角差值和距离差值，用于调试
    printf("diff_yaw: %f\n", diff_yaw);
    printf("diff_distance: %f\n", diff_distance);

    // 根据偏航角差值和最大速度限制，计算全局坐标系下的x和y方向的速度
    double vx_global = max_x_speed_ * cos(diff_yaw) * p_value_;
    double vy_global = max_y_speed_ * sin(diff_yaw) * p_value_;
    // 打印当前的偏航角和计算出的全局速度，用于调试
    std::cout << "yaw_" << yaw_ << std::endl;
    std::cout << "vx_gl  " << vx_global << "   vy_gl   " << vy_global << std::endl;

    // 将全局坐标系下的速度转换为机器人坐标系下的速度
    cmd_vel.linear.x = vx_global * cos(yaw_)  + vy_global * sin (yaw_);
    cmd_vel.linear.y = -vx_global * sin(yaw_) + vy_global * cos(yaw_);
    // 设置机器人的角速度为预设的偏航速度
    cmd_vel.angular.z = set_yaw_speed_;
}

// 全局路径回调函数
void RobotCtrl::GlobalPathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    if (!msg->poses.empty()) {
        global_path_ = *msg;
        prune_index_ = 0;
        plan_ = true;
    }
}

bool RobotCtrl::Follower_StateReq(const std::shared_ptr<rm_decision_interfaces::srv::PidPlannerStatus::Request> req,
                                  std::shared_ptr<rm_decision_interfaces::srv::PidPlannerStatus::Response> resp) {
    // 使用RCLCPP_INFO记录请求数据，包括planner_state、max_x_speed、max_y_speed和yaw_speed
    RCLCPP_INFO(this->get_logger(), "Request data : planner_state = %d, max_x_speed = %f, max_y_speed = %f, yaw_speed = %f",
                req->planner_state, req->max_x_speed, req->max_y_speed, req->yaw_speed);

    // 检查planner_state是否在有效范围内（0到2之间）
    if (req->planner_state < 0 || req->planner_state > 2) {
        // 如果planner_state不在有效范围内，记录错误信息并返回失败
        RCLCPP_ERROR(this->get_logger(), "Submitted data exception: Data cannot be negative");
        resp->result = 0;    // 失败时返回0
        return false;
    }

    // 将请求中的planner_state赋值给成员变量planner_state_
    planner_state_ = req->planner_state;

    // 检查max_x_speed和max_y_speed是否大于0，如果是则更新成员变量
    if (req->max_x_speed > 0 && req->max_y_speed > 0) {
        max_x_speed_ = req->max_x_speed;
        max_y_speed_ = req->max_y_speed;
    }
    // 检查yaw_speed是否大于0，如果是则更新成员变量set_yaw_speed_
    if (req->yaw_speed > 0) {
        set_yaw_speed_ = req->yaw_speed;
    }

    // 设置响应结果为成功（1）
    resp->result = 1;   // 成功时返回1

    // 返回true表示函数执行成功
    return true;
}

void RobotCtrl::GimbalCtrl() {
    // 初始化云台pitch轴位置为0
    // a_gimbal_pitch_position = 0;
    // 将云台yaw轴位置设置为成员变量yaw_的值
    a_gimbal_yaw_position = yaw_;

    // 创建一个Float64类型的消息对象
    std_msgs::msg::Float64 cmd;
    // 注释掉的代码用于发布云台pitch轴位置命令，但当前未使用
    // cmd.data = a_gimbal_pitch_position;
    // gimbal_pitch_position_cmd_->publish(cmd);
    
    // 设置消息数据为云台yaw轴位置
    cmd.data = a_gimbal_yaw_position;
    // 注释掉的代码用于发布云台yaw轴位置命令，但当前未使用
    // gimbal_yaw_position_cmd_->publish(cmd);
}
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(pid_position_follow::RobotCtrl)