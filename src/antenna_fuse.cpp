#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "farmbot_interfaces/msg/float32_stamped.hpp"
#include "farmbot_interfaces/srv/datum.hpp"
#include "farmbot_interfaces/srv/trigger.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

double toRadians(double degrees) {
    return degrees * M_PI / 180.0;
}

std::pair<float, float> calc_bearing(double lat1_in, double long1_in, double lat2_in, double long2_in, double angle_gpses = 0.0) {
    // Convert latitude and longitude to radians
    double lat1 = toRadians(lat1_in);
    double long1 = toRadians(long1_in);
    double lat2 = toRadians(lat2_in);
    double long2 = toRadians(long2_in);
    // Calculate the bearing
    double bearing_rad = atan2(
        sin(long2 - long1) * cos(lat2),
        cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(long2 - long1)
    );
    // Add 90 degrees to get the bearing from north
    bearing_rad += M_PI / 2.0;
    // Convert the bearing to degrees
    double bearing_deg = bearing_rad * 180.0 / M_PI;
    // Make sure the bearing is positive
    bearing_deg = fmod((bearing_deg + 360.0), 360.0);
    // Add the angle between the GPSes
    bearing_deg += angle_gpses;
    // Make sure bearing is between -2pi and 2pi
    if (bearing_rad < -M_PI) {
        bearing_rad += 2 * M_PI;
    } else if (bearing_rad >= M_PI) {
        bearing_rad -= 2 * M_PI;
    }
    return std::make_pair(static_cast<float>(bearing_deg), static_cast<float>(bearing_rad));
}

class AntennaFuse : public rclcpp::Node {
    private:
        sensor_msgs::msg::NavSatFix datum;
        sensor_msgs::msg::NavSatFix curr_pose;
        bool datum_set = false;
        double angle_gpses = 0.0;


        message_filters::Subscriber<sensor_msgs::msg::NavSatFix> gps_front_;
        message_filters::Subscriber<sensor_msgs::msg::NavSatFix> gps_back_;
        std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::NavSatFix, sensor_msgs::msg::NavSatFix>>> sync_;

        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
        rclcpp::Publisher<farmbot_interfaces::msg::Float32Stamped>::SharedPtr deg_;
        rclcpp::Publisher<farmbot_interfaces::msg::Float32Stamped>::SharedPtr rad_;
        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr dat_pub_;

        rclcpp::Service<farmbot_interfaces::srv::Datum>::SharedPtr datum_gps_;
        rclcpp::Service<farmbot_interfaces::srv::Trigger>::SharedPtr datum_set_;

    public:
        AntennaFuse() : Node(
            "antenna_fuse",
            rclcpp::NodeOptions()
            .allow_undeclared_parameters(true)
            .automatically_declare_parameters_from_overrides(true)
        ){
            RCLCPP_INFO(this->get_logger(), "Starting GPS Fuser");

            rclcpp::Parameter param_val = this->get_parameter("name"); 
            rclcpp::Parameter topic_prefix_param = this->get_parameter("topic_prefix");

            try{
                rclcpp::Parameter gps_front_param = this->get_parameter("gps_main");
                gps_front_.subscribe(this, gps_front_param.as_string());
                rclcpp::Parameter gps_back_param = this->get_parameter("gps_aux");
                gps_back_.subscribe(this, gps_back_param.as_string());
                rclcpp::Parameter angle_gpses_param = this->get_parameter("angle_gpses");
                angle_gpses = angle_gpses_param.as_double();
            } catch(const std::exception& e) {
                RCLCPP_INFO(this->get_logger(), "Error: %s", e.what());
                gps_front_.subscribe(this, topic_prefix_param.as_string() + "/gps_main");
                gps_back_.subscribe(this, topic_prefix_param.as_string() + "/gps_aux");
            }


            sync_ = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::NavSatFix, sensor_msgs::msg::NavSatFix>>>(10);
            sync_->connectInput(gps_front_, gps_back_);
            sync_->registerCallback(std::bind(&AntennaFuse::callback, this, std::placeholders::_1, std::placeholders::_2));

            gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(topic_prefix_param.as_string() + "/loc/fix", 10);
            dat_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(topic_prefix_param.as_string() + "/loc/ref", 10);
            deg_ = this->create_publisher<farmbot_interfaces::msg::Float32Stamped>(topic_prefix_param.as_string() + "/loc/deg", 10);
            rad_ = this->create_publisher<farmbot_interfaces::msg::Float32Stamped>(topic_prefix_param.as_string() + "/loc/rad", 10);

            datum_gps_ = this->create_service<farmbot_interfaces::srv::Datum>(topic_prefix_param.as_string() + "/datum", std::bind(&AntennaFuse::datum_gps_callback, this, std::placeholders::_1, std::placeholders::_2));
            datum_set_ = this->create_service<farmbot_interfaces::srv::Trigger>(topic_prefix_param.as_string() + "/datum/set", std::bind(&AntennaFuse::datum_set_callback, this, std::placeholders::_1, std::placeholders::_2));
        }

    private:

        void callback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr& gps_front_msg, const sensor_msgs::msg::NavSatFix::ConstSharedPtr& gps_back_msg) {
            curr_pose = *gps_front_msg;
            curr_pose.header.frame_id = "gps";
 
            farmbot_interfaces::msg::Float32Stamped deg_msg;
            farmbot_interfaces::msg::Float32Stamped rad_msg;
            deg_msg.header = curr_pose.header;
            rad_msg.header = curr_pose.header;
            auto [bearing_deg, bearing_rad] = calc_bearing(curr_pose.latitude, curr_pose.longitude, gps_back_msg->latitude, gps_back_msg->longitude);
            deg_msg.data = bearing_deg;
            rad_msg.data = bearing_rad;

            gps_pub_->publish(curr_pose);
            deg_->publish(deg_msg);
            rad_->publish(rad_msg);

            if (datum_set) { 
                datum.header = curr_pose.header;
                dat_pub_->publish(datum);
            }
        }

        void datum_gps_callback(const std::shared_ptr<farmbot_interfaces::srv::Datum::Request> _request, std::shared_ptr<farmbot_interfaces::srv::Datum::Response> _response) {
            RCLCPP_INFO(this->get_logger(), "Datum Set Request -> SPECIFIED");
            datum = _request->gps;
            datum_set = true;
            _response->message = "DATUM SET TO: " + std::to_string(datum.latitude) + ", " + std::to_string(datum.longitude) + ", " + std::to_string(datum.altitude);
            return;
        }

        void datum_set_callback(const std::shared_ptr<farmbot_interfaces::srv::Trigger::Request> _request, std::shared_ptr<farmbot_interfaces::srv::Trigger::Response> _response) {
            RCLCPP_INFO(this->get_logger(), "Datum Set Request -> CURRENT");
            auto req = _request;  // to avoid unused parameter warning
            auto res = _response; // to avoid unused parameter warning
            datum = curr_pose;
            datum_set = true;
            return;
        }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto navfix = std::make_shared<AntennaFuse>();
    rclcpp::spin(navfix);
    rclcpp::shutdown();
    return 0;
}
