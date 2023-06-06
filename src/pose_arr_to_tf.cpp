#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_broadcaster.h"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class PoseArrayToTf : public rclcpp::Node
{
    geometry_msgs::msg::Pose previous_pose;
    float speed_mps;
    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";
        for (const auto &param : parameters)
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "Param update: " << param.get_name().c_str() << ": " << param.value_to_string().c_str());
            if (param.get_name() == "pose_topic")
            {
                pose_topic = param.as_string();
            }
        }
        return result;
    }

public:
    PoseArrayToTf() : Node("waypoint_saver_node")
    {

        this->declare_parameter<std::string>("pose_topic", "");

        this->get_parameter("pose_topic", pose_topic);

        callback_handle_ = this->add_on_set_parameters_callback(std::bind(&PoseArrayToTf::parametersCallback, this, std::placeholders::_1));

        sub_pose_array_ = this->create_subscription<geometry_msgs::msg::PoseArray>(pose_topic, 10, std::bind(&PoseArrayToTf::poseArrayCallback, this, _1));
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

private:
    void poseArrayCallback(const geometry_msgs::msg::PoseArray &current_pose_arr)
    {
         - previous_pose.position.x;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "map";
        t.child_frame_id = "base_link";
        t.transform.translation.x = current_pose_arr.poses[1].position.x;
        t.transform.translation.y = current_pose_arr.poses[1].position.y;
        t.transform.rotation = current_pose_arr.poses[1].orientation;
        tf_broadcaster_->sendTransform(t);
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_pose_array_;
    std::string pose_topic;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    geometry_msgs::msg::TransformStamped t;
    OnSetParametersCallbackHandle::SharedPtr callback_handle_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseArrayToTf>());
    rclcpp::shutdown();
    return 0;
}