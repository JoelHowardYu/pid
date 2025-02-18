#include "pid_position_follow.hpp"

namespace pid_position_follow
{
RobotCtrl::RobotCtrl(const rclcpp::NodeOptions &options) : Node("pid_position_follow", options) {
    this->declare_parameter("max_x_speed", 1.0);
    this->declare_parameter("max_y_speed", 1.0);
    this->declare_parameter("set_yaw_speed", 0.0);
    this->declare_parameter("p_value", 0.5);
    this->declare_parameter("i_value", 1.0);
    this->declare_parameter("d_value", 1.0);
    this->declare_parameter("plan_frequency", 30);
    this->declare_parameter("goal_dist_tolerance", 0.2);
    this->declare_parameter("prune_ahead_distance", 0.5);
    this->declare_parameter("global_frame", "map");

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

    gimbal_yaw_position_cmd_ = this->create_publisher<std_msgs::msg::Float64>("/auto_car/yaw_steering_position_controller/command", 10);
    // gimbal_pitch_position_cmd_ = this->create_publisher<std_msgs::msg::Float64>("/auto_car/pitch_steering_position_controller/command", 10);
    local_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/local_path", 5);
    global_path_sub_ = this->create_subscription<nav_msgs::msg::Path>("/plan", 5, std::bind(&RobotCtrl::GlobalPathCallback, this, std::placeholders::_1));
    game_state_sub_ = this->create_subscription<rm_decision_interfaces::msg::GameStatus>("/game_state", 5, std::bind(&RobotCtrl::Game_StateCallback, this, std::placeholders::_1));
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/base_vel", 10);
    jointstate_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&RobotCtrl::JointstateCallback, this, std::placeholders::_1));

    planner_server_ = this->create_service<rm_decision_interfaces::srv::PidPlannerStatus>("/pid_planner_status", 
        [this](const std::shared_ptr<rm_decision_interfaces::srv::PidPlannerStatus::Request> req,
               std::shared_ptr<rm_decision_interfaces::srv::PidPlannerStatus::Response> resp) {
            return this->Follower_StateReq(req, resp);
        });

    // 创建一个服务，用于处理决策请求
    decision_server_ = 
        this->create_service<rm_decision_interfaces::srv::Decision>(
            "decision_server",
            [this](const std::shared_ptr<rm_decision_interfaces::srv::Decision::Request> request,
                std::shared_ptr<rm_decision_interfaces::srv::Decision::Response> response) {
                this->serverCallback(request, response);
            });

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    transform_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    plan_timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / plan_freq_), 
            [this]() {
                this->Plan(nullptr);
            });
}

// 服务回调函数，处理决策请求
void RobotCtrl::serverCallback(
    const std::shared_ptr<rm_decision_interfaces::srv::Decision::Request> request,
    std::shared_ptr<rm_decision_interfaces::srv::Decision::Response> response) 
    {
        RCLCPP_INFO(this->get_logger(),"serverCallback");
        // RCLCPP_INFO(this->get_logger(), "收到a: %ld b: %ld", request->a,
        //             request->b);
        // response->sum = request->a + request->b;
    }

void RobotCtrl::Plan([[maybe_unused]] const rclcpp::TimerBase::SharedPtr event) {
    if (game_state_ == 4 && planner_state_ == 2) {
        if (plan_) {
            auto begin = std::chrono::steady_clock::now();
            auto start = this->now();

            // 1. Update the transform from global path frame to local planner frame
            geometry_msgs::msg::TransformStamped transform_stamped;
            try {
                transform_stamped = tf_buffer_->lookupTransform(global_frame_, global_path_.header.frame_id, tf2::TimePointZero);
            } catch (tf2::TransformException &ex) {
                RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
                return;
            }

            // 2. Get current robot pose in global path frame
            geometry_msgs::msg::PoseStamped robot_pose;
            GetGlobalRobotPose(tf_buffer_, global_path_.header.frame_id, robot_pose);

            // 3. Check if robot has already arrived with given distance tolerance
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

            // 4. Get prune index from given global path
            FindNearstPose(robot_pose, global_path_, prune_index_, prune_ahead_dist_);

            // 5. Generate the prune path and transform it into local planner frame
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

            // 6. Generate the cubic spline trajectory from above prune path
            GenTraj(prune_path, local_path);
            local_path_pub_->publish(local_path);

            // 7. Follow the trajectory and calculate the velocity
            geometry_msgs::msg::Twist cmd_vel;
            FollowTraj(robot_pose, local_path, cmd_vel);
            cmd_vel_pub_->publish(cmd_vel);
            GimbalCtrl();

            auto plan_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - begin);
            RCLCPP_INFO(this->get_logger(), "Planning takes %f ms and passed %d/%d.",
                        plan_time.count() / 1000.,
                        prune_index_,
                        static_cast<int>(global_path_.poses.size()));
        } else {
            geometry_msgs::msg::Twist cmd_vel;
            cmd_vel.linear.x = 0;
            cmd_vel.linear.y = 0;
            cmd_vel.angular.z = set_yaw_speed_;
            cmd_vel.linear.z = 0;   // bool success or not
            cmd_vel_pub_->publish(cmd_vel);
            GimbalCtrl();
        }
    } else if (planner_state_ == 1) {
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = 0;
        cmd_vel.linear.y = 0;
        cmd_vel.angular.z = set_yaw_speed_;
        cmd_vel.linear.z = 0;   // bool success or not
        cmd_vel_pub_->publish(cmd_vel);
        GimbalCtrl();
    } else {
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = 0;
        cmd_vel.linear.y = 0;
        cmd_vel.angular.z = 0;
        cmd_vel.linear.z = 0;   // bool success or not
        cmd_vel_pub_->publish(cmd_vel);
        GimbalCtrl();
    }
}

void RobotCtrl::FindNearstPose(geometry_msgs::msg::PoseStamped& robot_pose, nav_msgs::msg::Path& path, int& prune_index, double prune_ahead_dist) {
    double dist_threshold = 10; // threshold is 10 meters (basically never over 10m i suppose)
    double sq_dist_threshold = dist_threshold * dist_threshold;
    double sq_dist;
    if (prune_index != 0) {
        sq_dist = GetEuclideanDistance(robot_pose, path.poses[prune_index - 1]);
    } else {
        sq_dist = 1e10;
    }

    double new_sq_dist = 0;
    while (prune_index < (int)path.poses.size()) {
        new_sq_dist = GetEuclideanDistance(robot_pose, path.poses[prune_index]);
        if (new_sq_dist > sq_dist && sq_dist < sq_dist_threshold) {
            // Judge if it is in the same direction and sq_dist is further than 0.3 meters
            if ((path.poses[prune_index].pose.position.x - robot_pose.pose.position.x) *
                (path.poses[prune_index - 1].pose.position.x - robot_pose.pose.position.x) +
                (path.poses[prune_index].pose.position.y - robot_pose.pose.position.y) *
                (path.poses[prune_index - 1].pose.position.y - robot_pose.pose.position.y) > 0
                && sq_dist > prune_ahead_dist) {
                prune_index--;
            } else {
                sq_dist = new_sq_dist;
            }

            break;
        }
        sq_dist = new_sq_dist;
        ++prune_index;
    }

    prune_index = std::min(prune_index, (int)(path.poses.size() - 1));
}

void RobotCtrl::FollowTraj(const geometry_msgs::msg::PoseStamped& robot_pose, const nav_msgs::msg::Path& traj, geometry_msgs::msg::Twist& cmd_vel) {
    geometry_msgs::msg::PoseStamped robot_pose_1;
    GetGlobalRobotPose(tf_buffer_, global_path_.header.frame_id, robot_pose_1);

    yaw_ = tf2::getYaw(robot_pose_1.pose.orientation);

    double diff_yaw = atan2((traj.poses[1].pose.position.y - robot_pose.pose.position.y), (traj.poses[1].pose.position.x - robot_pose.pose.position.x));

    double diff_distance = GetEuclideanDistance(robot_pose, traj.poses[1]);

    if (diff_yaw > M_PI) {
        diff_yaw -= 2 * M_PI;
    } else if (diff_yaw < -M_PI) {
        diff_yaw += 2 * M_PI;
    }

    printf("diff_yaw: %f\n", diff_yaw);
    printf("diff_distance: %f\n", diff_distance);

    double vx_global = max_x_speed_ * cos(diff_yaw) * p_value_;
    double vy_global = max_y_speed_ * sin(diff_yaw) * p_value_;
    std::cout << "yaw_" << yaw_ << std::endl;
    std::cout << "vx_gl  " << vx_global << "   vy_gl   " << vy_global << std::endl;

    cmd_vel.linear.x = vx_global * cos(yaw_) + vy_global * sin(yaw_);
    cmd_vel.linear.y = -vx_global * sin(yaw_) + vy_global * cos(yaw_);
    cmd_vel.angular.z = set_yaw_speed_;
}

void RobotCtrl::GlobalPathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    if (!msg->poses.empty()) {
        global_path_ = *msg;
        prune_index_ = 0;
        plan_ = true;
    }
}

void RobotCtrl::Game_StateCallback(const rm_decision_interfaces::msg::GameStatus::SharedPtr msg) {
    game_state_ = msg->game_state;
}

void RobotCtrl::JointstateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    cur_gimbal_yaw_position = msg->position[0];
    cur_gimbal_pitch_position = msg->position[1];
}

void RobotCtrl::ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    // int a;
    // yaw_ = tf2::getYaw(msg->orientation);
    // RCLCPP_INFO(this->get_logger(), "imu_yaw:%f", yaw_);
}

bool RobotCtrl::Follower_StateReq(const std::shared_ptr<rm_decision_interfaces::srv::PidPlannerStatus::Request> req,
                                  std::shared_ptr<rm_decision_interfaces::srv::PidPlannerStatus::Response> resp) {
    RCLCPP_INFO(this->get_logger(), "Request data : planner_state = %d, max_x_speed = %f, max_y_speed = %f, yaw_speed = %f",
                req->planner_state, req->max_x_speed, req->max_y_speed, req->yaw_speed);

    if (req->planner_state < 0 || req->planner_state > 2) {
        RCLCPP_ERROR(this->get_logger(), "Submitted data exception: Data cannot be negative");
        resp->result = 0;    // 失败时返回0
        return false;
    }

    planner_state_ = req->planner_state;

    if (req->max_x_speed > 0 && req->max_y_speed > 0) {
        max_x_speed_ = req->max_x_speed;
        max_y_speed_ = req->max_y_speed;
    }
    if (req->yaw_speed > 0) {
        set_yaw_speed_ = req->yaw_speed;
    }

    resp->result = 1;   // 成功时返回1

    return true;
}

void RobotCtrl::GimbalCtrl() {
    a_gimbal_pitch_position = 0;
    a_gimbal_yaw_position = yaw_;

    std_msgs::msg::Float64 cmd;
    // cmd.data = a_gimbal_pitch_position;
    // gimbal_pitch_position_cmd_->publish(cmd);
    cmd.data = a_gimbal_yaw_position;
    // gimbal_yaw_position_cmd_->publish(cmd);
}

}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(pid_position_follow::RobotCtrl)