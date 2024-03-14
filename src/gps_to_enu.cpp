#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "farmbot_interfaces/msg/float32_stamped.hpp"
#include "farmbot_interfaces/msg/float64_stamped.hpp"
#include "farmbot_interfaces/srv/datum.hpp"
#include "farmbot_interfaces/srv/trigger.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

const double R = 6356752.3142;
const double f_inv = 298.257223563;
const double f = 1.0 / f_inv;
const double e2 = 1 - (1 - f) * (1 - f);

std::tuple<double, double, double> gps_to_ecef(double latitude, double longitude, double altitude) {
    double cosLat = std::cos(latitude * M_PI / 180);
    double sinLat = std::sin(latitude * M_PI / 180);
    double cosLong = std::cos(longitude * M_PI / 180);
    double sinLong = std::sin(longitude * M_PI / 180);
    double c = 1 / std::sqrt(cosLat * cosLat + (1 - f) * (1 - f) * sinLat * sinLat);
    double s = (1 - f) * (1 - f) * c;
    double x = (R*c + altitude) * cosLat * cosLong;
    double y = (R*c + altitude) * cosLat * sinLong;
    double z = (R*s + altitude) * sinLat;
    return std::make_tuple(x, y, z);
}

std::tuple<double, double, double> ecef_to_enu(std::tuple<double, double, double> ecef, std::tuple<double, double, double> datum) {
    double x, y, z;
    std::tie(x, y, z) = ecef;
    double latRef, longRef, altRef;
    std::tie(latRef, longRef, altRef) = datum;
    double cosLatRef = std::cos(latRef * M_PI / 180);
    double sinLatRef = std::sin(latRef * M_PI / 180);
    double cosLongRef = std::cos(longRef * M_PI / 180);
    double sinLongRef = std::sin(longRef * M_PI / 180);
    double cRef = 1 / std::sqrt(cosLatRef * cosLatRef + (1 - f) * (1 - f) * sinLatRef * sinLatRef);
    double x0 = (R*cRef + altRef) * cosLatRef * cosLongRef;
    double y0 = (R*cRef + altRef) * cosLatRef * sinLongRef;
    double z0 = (R*cRef*(1-e2) + altRef) * sinLatRef;
    double xEast = (-(x-x0) * sinLongRef) + ((y-y0)*cosLongRef);
    double yNorth = (-cosLongRef*sinLatRef*(x-x0)) - (sinLatRef*sinLongRef*(y-y0)) + (cosLatRef*(z-z0));
    double zUp = (cosLatRef*cosLongRef*(x-x0)) + (cosLatRef*sinLongRef*(y-y0)) + (sinLatRef*(z-z0));
    return std::make_tuple(xEast, yNorth, zUp);
}

class Gps2Enu : public rclcpp::Node {
    private:
        nav_msgs::msg::Odometry datum;
        bool datum_set = false;

        message_filters::Subscriber<sensor_msgs::msg::NavSatFix> fix_sub_;
        message_filters::Subscriber<sensor_msgs::msg::NavSatFix> ref_sub;
        std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::NavSatFix, sensor_msgs::msg::NavSatFix>>> sync_;

        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ecef_pub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr enu_pub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ecef_datum_pub_;

    public:
        Gps2Enu() : Node(
            "gps_to_enu",
            rclcpp::NodeOptions()
            .allow_undeclared_parameters(true)
            .automatically_declare_parameters_from_overrides(true)
        ){
            RCLCPP_INFO(this->get_logger(), "Starting GPS2ENU Node");

            rclcpp::Parameter param_val = this->get_parameter("name"); 
            rclcpp::Parameter topic_prefix_param = this->get_parameter("topic_prefix");

            fix_sub_.subscribe(this, topic_prefix_param.as_string() + "/loc/fix");
            ref_sub.subscribe(this, topic_prefix_param.as_string() + "/loc/ref");
            sync_ = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::NavSatFix, sensor_msgs::msg::NavSatFix>>>(10);
            sync_->connectInput(fix_sub_, ref_sub);
            sync_->registerCallback(std::bind(&Gps2Enu::callback, this, std::placeholders::_1, std::placeholders::_2));

            ecef_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(topic_prefix_param.as_string() + "/loc/ecef", 10);
            enu_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(topic_prefix_param.as_string() + "/loc/enu", 10);
            ecef_datum_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(topic_prefix_param.as_string() + "/loc/ref/ecef", 10);
        }

    private:
        void callback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr& fix, const sensor_msgs::msg::NavSatFix::ConstSharedPtr& ref) {
            if (!datum_set) {
                set_ecef_datum(ref);
                datum_set = true;
            } else {
                datum.header = ref->header;
                ecef_datum_pub_->publish(datum);
            }

            double lat = fix->latitude, lon = fix->longitude, alt = fix->altitude;
            nav_msgs::msg::Odometry ecef_msg;
            ecef_msg.header = fix->header;
            double ecef_x, ecef_y, ecef_z;
            std::tie(ecef_x, ecef_y, ecef_z) = gps_to_ecef(lat, lon, alt);
            ecef_msg.pose.pose.position.x = ecef_x;
            ecef_msg.pose.pose.position.y = ecef_y;
            ecef_msg.pose.pose.position.z = ecef_z;

            nav_msgs::msg::Odometry enu_msg;
            enu_msg.header = fix->header;
            double d_lat = ref->latitude, d_lon = ref->longitude, d_alt = ref->altitude;
            double enu_x, enu_y, enu_z;
            std::tie(enu_x, enu_y, enu_z) = ecef_to_enu(std::make_tuple(ecef_x, ecef_y, ecef_z), std::make_tuple(d_lat, d_lon, d_alt));
            enu_msg.pose.pose.position.x = enu_x;
            enu_msg.pose.pose.position.y = enu_y;
            enu_msg.pose.pose.position.z = enu_z;

            ecef_pub_->publish(ecef_msg);
            enu_pub_->publish(enu_msg);
        }

        void set_ecef_datum(const sensor_msgs::msg::NavSatFix::ConstSharedPtr& ref) {
            datum.header = ref->header;
            double lat = ref->latitude;
            double lon = ref->longitude;
            double alt = ref->altitude;
            double x, y, z;
            std::tie(x, y, z) = gps_to_ecef(lat, lon, alt);
            datum.pose.pose.position.x = x;
            datum.pose.pose.position.y = y;
            datum.pose.pose.position.z = z;
        }

};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto navfix = std::make_shared<Gps2Enu>();
    rclcpp::spin(navfix);
    rclcpp::shutdown();
    return 0;
}
