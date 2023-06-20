// publishes nav_msgs/Path and steering marker and km/h

#include <iostream>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/convert.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

class Visuals : public rclcpp::Node
{
public:
    Visuals() : Node("visuals_node")
    {
        this->declare_parameter<std::string>("pose_frame", "base_link");
        this->declare_parameter<std::string>("marker_topic", "marker_steering");
        this->declare_parameter<std::string>("path_topic", "marker_path");
        this->declare_parameter<bool>("publish_steer_marker", true);
        this->declare_parameter<int>("path_size", 1500);
        this->declare_parameter<int>("mod_limit", 10); // modulo limit for path size (publish every n-th message)

        this->get_parameter("pose_frame", pose_frame);
        this->get_parameter("marker_topic", marker_topic);
        this->get_parameter("path_topic", path_topic);
        this->get_parameter("publish_steer_marker", publish_steer_marker);
        this->get_parameter("path_size", path_size);
        this->get_parameter("mod_limit", mod_limit);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        marker_pub = this->create_publisher<visualization_msgs::msg::Marker>(marker_topic, 1);
        path_pub = this->create_publisher<nav_msgs::msg::Path>(path_topic, 1);
        sub_twist_ = this->create_subscription<geometry_msgs::msg::Twist>("/model/vehicle_blue/cmd_vel", 10, std::bind(&Visuals::vehicleTwistCallback, this, _1));
        // Call loop function 20 Hz (50 milliseconds)
        timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&Visuals::loop, this));
        RCLCPP_INFO_STREAM(this->get_logger(), "Node started: " << this->get_name());
    }

private:

    // Callback for steering wheel and speed cmd messages
    void vehicleTwistCallback(const geometry_msgs::msg::Twist &vehicle_msg)
    {
        steering_angle = vehicle_msg.angular.z;
        vehicle_speed_mps = vehicle_msg.linear.x;
    }

    // get tf2 transform from map to base_link
    void vehiclePoseFromTransform()
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        try
        {
            transformStamped = tf_buffer_->lookupTransform("map", pose_frame, tf2::TimePointZero);
        }

        catch (const tf2::TransformException &ex)
        {
            // RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
            return;
        }
        actual_pose.pose.position.x = transformStamped.transform.translation.x;
        actual_pose.pose.position.y = transformStamped.transform.translation.y;
        actual_pose.pose.position.z = transformStamped.transform.translation.z + 1.0;
        actual_pose.pose.orientation.x = transformStamped.transform.rotation.x;
        actual_pose.pose.orientation.y = transformStamped.transform.rotation.y;
        actual_pose.pose.orientation.z = transformStamped.transform.rotation.z;
        actual_pose.pose.orientation.w = transformStamped.transform.rotation.w;
        actual_pose.header.stamp = this->now();
        actual_pose.header.frame_id = "map";
        // RCLCPP_INFO_STREAM(this->get_logger(), "actual_pose: " << actual_pose.pose.position.x << ", " << actual_pose.pose.position.y << ", " << actual_pose.pose.position.z);
    }

    void loop()
    {
        vehiclePoseFromTransform();
        if (publish_steer_marker)
        {
            visualization_msgs::msg::Marker steer_marker;
            steer_marker.header.frame_id = "/base_link";
            steer_marker.header.stamp = this->now();
            steer_marker.ns = "steering_path";
            steer_marker.id = 0;
            steer_marker.type = steer_marker.LINE_STRIP;
            steer_marker.action = visualization_msgs::msg::Marker::MODIFY;
            steer_marker.pose.position.x = 0;
            steer_marker.pose.position.y = 0;
            steer_marker.pose.position.z = 0;
            steer_marker.pose.orientation.x = 0.0;
            steer_marker.pose.orientation.y = 0.0;
            steer_marker.pose.orientation.z = 0.0;
            steer_marker.pose.orientation.w = 1.0;
            steer_marker.scale.x = 0.6;
            // https://github.com/jkk-research/colors
            steer_marker.color.r = 0.94f;
            steer_marker.color.g = 0.83f;
            steer_marker.color.b = 0.07f;
            steer_marker.color.a = 1.0;
            steer_marker.lifetime = rclcpp::Duration::from_seconds(0);
            double marker_pos_x = 0.0, marker_pos_y = 0.0, theta = 0.0;
            for (int i = 0; i < 80; i++)
            {
                // Ackermann steering model
                marker_pos_x += 0.01 * 10 * cos(theta);
                marker_pos_y += 0.01 * 10 * sin(theta);
                theta += 0.01 * 10 / wheelbase * tan(steering_angle);
                geometry_msgs::msg::Point p;
                p.x = marker_pos_x;
                p.y = marker_pos_y;
                steer_marker.points.push_back(p);
            }
            marker_pub->publish(steer_marker);
            steer_marker.points.clear();
        }
        geometry_msgs::msg::PoseStamped pose;

        pose.header.stamp = this->now();
        pose.pose.position = actual_pose.pose.position;
        pose.header.frame_id = "map";
        path.header.frame_id = "map";
        pose.pose.orientation = actual_pose.pose.orientation;
        path.poses.push_back(pose);
        path.header.stamp = this->now();
        if (loop_increment % mod_limit == 0){
            path.poses.push_back(pose);
        }
        // keep only the last n (path_size) path message
        if (path.poses.size() > path_size)
        {
            int shift = path.poses.size() - path_size;
            path.poses.erase(path.poses.begin(), path.poses.begin() + shift);
        }

        path_pub->publish(path);
        loop_increment += 1;
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_;
    std::string pose_topic, marker_topic, path_topic, pose_frame;
    const double wheelbase = 1.0;
    double steering_angle, vehicle_speed_mps;
    long unsigned int path_size;
    bool steering_enabled;
    bool first_run = true, publish_steer_marker, publish_kmph;
    std::string marker_color;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
    nav_msgs::msg::Path path;
    geometry_msgs::msg::PoseStamped actual_pose;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    int loop_increment = 0, mod_limit = 20;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Visuals>());
    rclcpp::shutdown();
    return 0;
}