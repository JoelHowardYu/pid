#ifndef ROBOMASTER_UTILITY_H
#define ROBOMASTER_UTILITY_H

#include <tf2/utils.h>
#include <tf2_ros/buffer.h>  
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <rclcpp/rclcpp.hpp>

double GetYawFromOrientation(const geometry_msgs::msg::Quaternion &orientation) {
    tf2::Quaternion q;
    tf2::fromMsg(orientation, q);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

double GetEuclideanDistance(const geometry_msgs::msg::PoseStamped &pose_1,
                            const geometry_msgs::msg::PoseStamped &pose_2) {
    return hypot(pose_1.pose.position.x - pose_2.pose.position.x,
                 pose_1.pose.position.y - pose_2.pose.position.y);
}

bool GetGlobalRobotPose(const std::shared_ptr<tf2_ros::Buffer> &tf_buffer,
                        const std::string &target_frame,
                        geometry_msgs::msg::PoseStamped &robot_global_pose) {
    geometry_msgs::msg::PoseStamped robot_pose;
    robot_pose.header.frame_id = "base_link";
    robot_pose.header.stamp = tf2_ros::toMsg(tf2::TimePointZero);
    robot_pose.pose.orientation.w = 1.0;

    try {
        tf_buffer->transform(robot_pose, robot_global_pose, target_frame);
    } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(rclcpp::get_logger("utility"), "Failed to transform robot pose: %s", ex.what());
        return false;
    }
    return true;
}

bool TransformPose(const std::shared_ptr<tf2_ros::Buffer> &tf_buffer,
                   const std::string &target_frame,
                   const geometry_msgs::msg::PoseStamped &input_pose,
                   geometry_msgs::msg::PoseStamped &output_pose) {
    if (target_frame == input_pose.header.frame_id) {
        output_pose = input_pose;
        return true;
    }

    try {
        tf_buffer->transform(input_pose, output_pose, target_frame);
    } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(rclcpp::get_logger("utility"), "Failed to transform pose: %s", ex.what());
        return false;
    }

    return true;
}

bool TransformPose(const std::shared_ptr<tf2_ros::Buffer> &tf_buffer,
                   const std::string &target_frame,
                   const geometry_msgs::msg::PoseStamped &input_pose,
                   geometry_msgs::msg::TransformStamped &output_pose_tf) {
    if (target_frame == input_pose.header.frame_id) {
        tf2::fromMsg(input_pose.pose, output_pose_tf.transform);
        output_pose_tf.header = input_pose.header;
        return true;
    }

    geometry_msgs::msg::PoseStamped output_pose;
    if (!TransformPose(tf_buffer, target_frame, input_pose, output_pose)) {
        return false;
    }

    tf2::fromMsg(output_pose.pose, output_pose_tf.transform);
    output_pose_tf.header = output_pose.header;
    return true;
}

void TransformPose(const geometry_msgs::msg::TransformStamped &transform,
                   const geometry_msgs::msg::PoseStamped &input_pose,
                   geometry_msgs::msg::PoseStamped &output_pose) {
    tf2::Transform input_pose_tf;
    tf2::fromMsg(input_pose.pose, input_pose_tf);
    tf2::Transform transform_tf;
    tf2::fromMsg(transform.transform, transform_tf);
    tf2::Transform output_pose_tf = transform_tf * input_pose_tf;
    // Convert tf2::Transform to geometry_msgs::msg::Pose
    geometry_msgs::msg::Pose output_pose_msg;
    tf2::toMsg(output_pose_tf, output_pose_msg);
    output_pose.pose = output_pose_msg;
    output_pose.header.stamp = transform.header.stamp;
    output_pose.header.frame_id = transform.header.frame_id;
}

bool UpdateTransform(const std::shared_ptr<tf2_ros::Buffer> &tf_buffer,
                     const std::string &target_frame,
                     const std::string &source_frame,
                     const rclcpp::Time &source_time,
                     geometry_msgs::msg::TransformStamped &target_to_source_transform) {
    try {
        target_to_source_transform = tf_buffer->lookupTransform(target_frame, source_time, source_frame, source_time, source_frame);
    } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(rclcpp::get_logger("utility"), "Failed to update transform: %s", ex.what());
        return false;
    }

    return true;
}

#endif // ROBOMASTER_UTILITY_H