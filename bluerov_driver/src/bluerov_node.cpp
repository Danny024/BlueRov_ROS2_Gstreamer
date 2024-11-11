/**
 * Name : Daniel Eneh
 * Date : 29-10-2024
 * Email : danieleneh024@gmail.com
 * 
 * 
 */

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

class BlueRov : public rclcpp::Node {
public:
    BlueRov()
    : Node("blue_rov_node"),
      custom_qos_profile(10)  // Initialize QoS with depth
    {
        // Set custom QoS settings
        custom_qos_profile.reliability(rclcpp::ReliabilityPolicy::Reliable)
                          .durability(rclcpp::DurabilityPolicy::TransientLocal);

        // Create Reentrant Callback Group
        auto callback_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        // Service initialization with callback groups
        arm_service_ = create_service<std_srvs::srv::SetBool>(
            "/bluerov2/arm",
            std::bind(&BlueRov::arm_callback, this, std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default,
            callback_group
        );

        disarm_service_ = create_service<std_srvs::srv::SetBool>(
            "/bluerov2/disarm",
            std::bind(&BlueRov::disarm_callback, this, std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default,
            callback_group
        );

        // Publisher initialization with custom QoS
        pub_topside_heartbeat_ = create_publisher<std_msgs::msg::Bool>(
            "/bluerov2/topside_heartbeat",
            custom_qos_profile
        );

        pub_odometry_ = create_publisher<nav_msgs::msg::Odometry>(
            "/bluerov2/odometry",
            custom_qos_profile
        );

        pub_battery_state_ = create_publisher<sensor_msgs::msg::BatteryState>(
            "/bluerov2/battery",
            custom_qos_profile
        );

        // Timers
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&BlueRov::publish_heartbeat, this)
        );

        RCLCPP_INFO(this->get_logger(), "BlueRov Node Initialized");
    }

private:
    // QoS Profile
    rclcpp::QoS custom_qos_profile;

    // Services
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr arm_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr disarm_service_;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_topside_heartbeat_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometry_;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr pub_battery_state_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Callback for the arm service
    void arm_callback(const std_srvs::srv::SetBool::Request::SharedPtr request,
                      std_srvs::srv::SetBool::Response::SharedPtr response) {
        RCLCPP_INFO(this->get_logger(), "Arm request received: %s", request->data ? "True" : "False");
        response->success = true;
        response->message = "ARM command received";
    }

    // Callback for the disarm service
    void disarm_callback(const std_srvs::srv::SetBool::Request::SharedPtr request,
                         std_srvs::srv::SetBool::Response::SharedPtr response) {
        RCLCPP_INFO(this->get_logger(), "Disarm request received: %s", request->data ? "True" : "False");
        response->success = true;
        response->message = "DISARM command received";
    }

    // Timer callback to publish heartbeat
    void publish_heartbeat() {
        auto heartbeat_msg = std_msgs::msg::Bool();
        heartbeat_msg.data = true;
        pub_topside_heartbeat_->publish(heartbeat_msg);

        RCLCPP_INFO(this->get_logger(), "Heartbeat published");
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BlueRov>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
