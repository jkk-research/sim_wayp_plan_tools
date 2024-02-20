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
#include <fstream>

using namespace std::chrono_literals;
using std::placeholders::_1;

class CsvSave : public rclcpp::Node
{
    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";
        for (const auto &param : parameters)
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "Param update: " << param.get_name().c_str() << ": " << param.value_to_string().c_str());
            if (param.get_name() == "pose_frame")
            {
                pose_frame = param.as_string();
            }
            if (param.get_name() == "metrics_topic")
            {
                metrics_topic = param.as_string();
            }
            if (param.get_name() == "file_name")
            {
                file_name = param.as_string();
            }
            if (param.get_name() == "file_dir")
            {
                file_dir = param.as_string();
            }
            if (param.get_name() == "mod_limit")
            {
                mod_limit = param.as_int();
            }
            file_path_xy = file_dir + "/" + file_name + "_xypose.csv";
            file_path_metrics = file_dir + "/" + file_name + "_metrics.csv";
        }
        return result;
    }

public:
    CsvSave() : Node("csv_saver_node")
    {
        this->declare_parameter<std::string>("pose_frame", "base_link");
        this->declare_parameter<std::string>("file_name", "tmp01");
        this->declare_parameter<std::string>("file_dir", "/home/");
        this->declare_parameter<std::string>("metrics_topic", "marker_steering");
        this->declare_parameter<int>("mod_limit", 10); // modulo limit for path size

        this->get_parameter("pose_frame", pose_frame);
        this->get_parameter("metrics_topic", metrics_topic);
        this->get_parameter("file_name", file_name);
        this->get_parameter("file_dir", file_dir);
        this->get_parameter("mod_limit", mod_limit);

        file_path_xy = file_dir + "/" + file_name + "_xypose.csv";
        file_path_metrics = file_dir + "/" + file_name + "_metrics.csv";
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        sub_twist_ = this->create_subscription<geometry_msgs::msg::Twist>("/model/vehicle_blue/cmd_vel", 10, std::bind(&CsvSave::vehicleTwistCallback, this, _1));
        // Call loop function 20 Hz (50 milliseconds)
        timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&CsvSave::loop, this));
        callback_handle_ = this->add_on_set_parameters_callback(std::bind(&CsvSave::parametersCallback, this, std::placeholders::_1));
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
        if (first_run)
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "CSV xy path: " << file_path_xy);
            start_time = this->now();
            first_run = false;
        }
        vehiclePoseFromTransform();
        geometry_msgs::msg::PoseStamped pose;

        if (loop_increment % mod_limit == 0)
        {
            // 3 decmial places for x, y
            std::ofstream file;
            file.open(file_path_xy, std::ios_base::app);
            double actual_time = (this->now() - start_time).seconds();
            file << std::fixed << std::setprecision(3) << actual_time << "," << actual_pose.pose.position.x << "," << actual_pose.pose.position.y << "," << steering_angle << "," << vehicle_speed_mps << "\n";
            // RCLCPP_INFO_STREAM(this->get_logger(), "XY:" << actual_pose.pose.position.x << "," << actual_pose.pose.position.y);
        }

        loop_increment += 1;
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_;
    OnSetParametersCallbackHandle::SharedPtr callback_handle_;
    std::string pose_topic, metrics_topic, path_topic, pose_frame, file_name, file_dir, file_path_xy, file_path_metrics;
    double steering_angle, vehicle_speed_mps;
    bool first_run = true;
    geometry_msgs::msg::PoseStamped actual_pose;
    rclcpp::Time start_time;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    int loop_increment = 0, mod_limit = 20;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CsvSave>());
    rclcpp::shutdown();
    return 0;
}