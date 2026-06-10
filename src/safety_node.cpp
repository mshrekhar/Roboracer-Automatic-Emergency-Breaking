#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

#include <cmath>
#include <limits>
#include <vector>
#include <deque>

class Safety : public rclcpp::Node {
public:
    Safety() : Node("safety_node") {
        using std::placeholders::_1;

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&Safety::scan_callback, this, _1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ego_racecar/odom", 10, std::bind(&Safety::drive_callback, this, _1));

        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "/drive", 10);
    }

private:
    // Current speed
    double speed_ = 0.0;

    // Subscribers & publisher
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;

    // Parameters
    const double ttc_threshold_  = 0.7;   // primary TTC threshold (s)
    const double sec_ttc_threshold_ = 0.1;   // sec_TTC threshold (s) for moving obstacles
    const int    sec_ttc_window_    = 15;    // look at last 15 scans
    const int    sec_ttc_min_hits_  = 13;     // require >=13 scans with sec_TTC < threshold

    std::vector<float> prev_ranges_;
    double prev_stamp_ = 0.0;
    bool have_prev_scan_ = false;

    std::deque<bool> sec_ttc_danger_history_;

    void drive_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        speed_ = msg->twist.twist.linear.x;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {
        const auto &ranges = scan_msg->ranges;
        const size_t n = ranges.size();

        // Primary TTC (speed-based, instantaneous)
        double min_ttc = std::numeric_limits<double>::infinity();

        if (std::abs(speed_) > 1e-3) {
            for (size_t i = 0; i < n; ++i) {
                float r = ranges[i];
                if (!std::isfinite(r) ||
                    r <= scan_msg->range_min ||
                    r >= scan_msg->range_max) {
                    continue;
                }

                double angle = scan_msg->angle_min + i * scan_msg->angle_increment;
                double r_dot = speed_ * std::cos(angle);  // projection of ego speed

                if (r_dot <= 0.0) {
                    continue;
                }

                double ttc = r / r_dot;
                if (ttc < min_ttc) {
                    min_ttc = ttc;
                }
            }
        }

        // sec_TTC (range-rate-based) for moving obstacles, using consecutive scans 
        double min_sec_ttc = std::numeric_limits<double>::infinity();
        bool   sec_ttc_danger_this_scan = false;

        double t = scan_msg->header.stamp.sec +
                   scan_msg->header.stamp.nanosec * 1e-9;

        if (have_prev_scan_ && prev_ranges_.size() == n) {
            double dt = t - prev_stamp_;
            if (dt > 0.0) {
                for (size_t i = 0; i < n; ++i) {
                    float r      = ranges[i];
                    float r_prev = prev_ranges_[i];

                    if (!std::isfinite(r) ||
                        r <= scan_msg->range_min ||
                        r >= scan_msg->range_max ||
                        !std::isfinite(r_prev)) {
                        continue;
                    }

                    // Range rate
                    double r_dot = (r - r_prev) / dt;
                    double closing_rate = -r_dot;  // positive when moving closer

                    if (closing_rate <= 0.0) {
                        continue;
                    }

                    double sec_ttc = r / closing_rate;
                    if (sec_ttc < min_sec_ttc) {
                        min_sec_ttc = sec_ttc;
                    }
                }
            }
        }

        // Update history for next scan (for sec_TTC)
        prev_ranges_ = ranges;
        prev_stamp_ = t;
        have_prev_scan_ = true;

        // Update sec_TTC danger history (only if we have a finite sec_TTC)
        if (std::isfinite(min_sec_ttc) && min_sec_ttc < sec_ttc_threshold_) {
            sec_ttc_danger_this_scan = true;
        }
        sec_ttc_danger_history_.push_back(sec_ttc_danger_this_scan);
        if (static_cast<int>(sec_ttc_danger_history_.size()) > sec_ttc_window_) {
            sec_ttc_danger_history_.pop_front();
        }

        int sec_ttc_hits = 0;
        for (bool b : sec_ttc_danger_history_) {
            if (b) sec_ttc_hits++;
        }
        bool sec_ttc_consistent_danger = (sec_ttc_hits >= sec_ttc_min_hits_);

        // Decision logic:
        // - Always respect primary TTC.
        // - Only give sec_TTC influence if it has been consistently dangerous.
        bool brake_by_ttc  = (min_ttc   < ttc_threshold_);
        bool brake_by_sec_ttc = sec_ttc_consistent_danger;

        if (brake_by_ttc || brake_by_sec_ttc) {
            RCLCPP_INFO(this->get_logger(),
                        "BRAKING! TTC=%.3f s, sec_TTC=%.3f s, sec_TTC_hits=%d/%d, speed=%.2f m/s",
                        std::isfinite(min_ttc) ? min_ttc : -1.0,
                        std::isfinite(min_sec_ttc) ? min_sec_ttc : -1.0,
                        sec_ttc_hits, sec_ttc_window_,
                        speed_);

            ackermann_msgs::msg::AckermannDriveStamped drive_msg;
            drive_msg.header.stamp = this->now();
            drive_msg.header.frame_id = "base_link";

            drive_msg.drive.speed = 0.0;
            drive_msg.drive.steering_angle = 0.0;

            drive_pub_->publish(drive_msg);
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Safety>());
    rclcpp::shutdown();
    return 0;
}

