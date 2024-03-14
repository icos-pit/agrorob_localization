#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "farmbot_interfaces/msg/float32_stamped.hpp"
#include "farmbot_interfaces/srv/datum.hpp"
#include "farmbot_interfaces/srv/trigger.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

std::array<double, 4> theta_to_quaternion(double theta) {
    return {std::cos(theta/2), 0, 0, -std::sin(theta/2)};
}


class OdomNPath : public rclcpp::Node {
    private:
        nav_msgs::msg::Odometry enu_odom;
        nav_msgs::msg::Odometry odom_map;
        nav_msgs::msg::Path path;
        bool reset_path = false;
        geometry_msgs::msg::PoseStamped prev_point_path;
        geometry_msgs::msg::PoseStamped prev_point_dist;
        farmbot_interfaces::msg::Float32Stamped cumulative_dist;

        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        rclcpp::TimerBase::SharedPtr odom_timer_;
        rclcpp::Publisher<farmbot_interfaces::msg::Float32Stamped>::SharedPtr dist_pub_;
        rclcpp::TimerBase::SharedPtr dist_timer_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
        rclcpp::TimerBase::SharedPtr path_timer_;
        rclcpp::Service<farmbot_interfaces::srv::Trigger>::SharedPtr dist_reset;
        rclcpp::Service<farmbot_interfaces::srv::Trigger>::SharedPtr path_reset;

        message_filters::Subscriber<nav_msgs::msg::Odometry> enu_sub_;
        message_filters::Subscriber<farmbot_interfaces::msg::Float32Stamped> rad_sub_;
        message_filters::Subscriber<nav_msgs::msg::Odometry> ecef_sub_;
        std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry, farmbot_interfaces::msg::Float32Stamped, nav_msgs::msg::Odometry>>> sync_;

    public:
        OdomNPath() : Node(
            "odom_n_path",
            rclcpp::NodeOptions()
            .allow_undeclared_parameters(true)
            .automatically_declare_parameters_from_overrides(true)
        ){
            RCLCPP_INFO(this->get_logger(), "Starting Odom&Path");
            const char* ns = this->get_namespace();
            rclcpp::Parameter param_val = this->get_parameter("name"); 
            rclcpp::Parameter topic_prefix_param = this->get_parameter("topic_prefix");

            odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(topic_prefix_param.as_string() + "/loc/odom", 10);
            odom_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&OdomNPath::odom_callback, this));
            dist_pub_ = this->create_publisher<farmbot_interfaces::msg::Float32Stamped>(topic_prefix_param.as_string() + "/loc/dist", 10);
            dist_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&OdomNPath::dist_callback, this));
            dist_reset = this->create_service<farmbot_interfaces::srv::Trigger>(topic_prefix_param.as_string() + "/loc/dist_reset", std::bind(&OdomNPath::dist_reset_callback, this, std::placeholders::_1, std::placeholders::_2));
            path_pub_ = this->create_publisher<nav_msgs::msg::Path>(topic_prefix_param.as_string() + "/loc/path", 10);
            path_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&OdomNPath::path_callback, this));
            path_reset = this->create_service<farmbot_interfaces::srv::Trigger>(topic_prefix_param.as_string() + "/loc/path_reset", std::bind(&OdomNPath::path_reset_callback, this, std::placeholders::_1, std::placeholders::_2));

            enu_sub_.subscribe(this, topic_prefix_param.as_string() + "/loc/enu");
            rad_sub_.subscribe(this, topic_prefix_param.as_string() + "/loc/rad");
            ecef_sub_.subscribe(this, topic_prefix_param.as_string() + "/loc/ecef");
            sync_ = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry, farmbot_interfaces::msg::Float32Stamped, nav_msgs::msg::Odometry>>>(10);
            sync_->connectInput(enu_sub_, rad_sub_, ecef_sub_);
            sync_->registerCallback(std::bind(&OdomNPath::callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
        }

    private:
        void callback(const nav_msgs::msg::Odometry::ConstSharedPtr& enu_msg, const farmbot_interfaces::msg::Float32Stamped::ConstSharedPtr& rad_msg, const nav_msgs::msg::Odometry::ConstSharedPtr& ecef_msg) {
            nav_msgs::msg::Odometry ecef_msg_ = *ecef_msg; // TODO: fix, this is a hack to get rid of unused variable warning
            enu_odom = *enu_msg;
            std::array<double, 4> quaterions = theta_to_quaternion(rad_msg->data);
            // RCLCPP_INFO(this->get_logger(), "quat: %.15f, %.15f, %.15f, %.15f", quaterions[0], quaterions[1], quaterions[2], quaterions[3]);
            enu_odom.pose.pose.orientation.w = quaterions[0];
            enu_odom.pose.pose.orientation.x = quaterions[1];
            enu_odom.pose.pose.orientation.y = quaterions[2];
            enu_odom.pose.pose.orientation.z =  quaterions[3];

            geometry_msgs::msg::PoseStamped pose;
            pose.header = enu_odom.header;
            pose.header.frame_id = "map";
            enu_odom.pose.pose.position.z = 0; // TODO: remove if you want to use altitude
            pose.pose = enu_odom.pose.pose;

            //create path
            path.header.frame_id = "map";
            if (prev_point_path.pose.position.x == 0) {
                prev_point_path = pose;
            }
            float distance_path = std::sqrt(std::pow(pose.pose.position.x - prev_point_path.pose.position.x, 2) + std::pow(pose.pose.position.y - prev_point_path.pose.position.y, 2));
            if (distance_path > 0.1) {
                prev_point_path = pose;
                path.poses.push_back(pose);
            }
            if (path.poses.size() > 100 && reset_path) {
                path.poses.erase(path.poses.begin());
            }

            //create dist
            if (prev_point_dist.pose.position.x == 0) {
                prev_point_dist = pose;
            }
            float distance_dist = std::sqrt(std::pow(pose.pose.position.x - prev_point_dist.pose.position.x, 2) + std::pow(pose.pose.position.y - prev_point_dist.pose.position.y, 2));
            if (distance_dist > 0.1) {
                prev_point_dist = pose;
                cumulative_dist.data += distance_dist;
                cumulative_dist.header = pose.header;
            }
            
        }

        void odom_callback() {
            odom_pub_->publish(enu_odom);
        }
        
        void dist_callback() {
            dist_pub_->publish(cumulative_dist);
        }

        void path_callback() {
            path_pub_->publish(path);
        }

        void dist_reset_callback(const std::shared_ptr<farmbot_interfaces::srv::Trigger::Request> _request, std::shared_ptr<farmbot_interfaces::srv::Trigger::Response> _response) {
            cumulative_dist.data = 0;
            auto req = _request; // TODO: fix, this is a hack to get rid of unused variable warning
            auto res = _response; // TODO: fix, this is a hack to get rid of unused variable warning
            return;
        }
        void path_reset_callback(const std::shared_ptr<farmbot_interfaces::srv::Trigger::Request> _request, std::shared_ptr<farmbot_interfaces::srv::Trigger::Response> _response) {
            path.poses.clear();
            auto req = _request; // TODO: fix, this is a hack to get rid of unused variable warning
            auto res = _response; // TODO: fix, this is a hack to get rid of unused variable warning
            return;
        }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto navfix = std::make_shared<OdomNPath>();
    rclcpp::spin(navfix);
    rclcpp::shutdown();
    return 0;
}
