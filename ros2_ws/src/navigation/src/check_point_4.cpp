#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>

struct PathCommand {
    double x, y;         // 下一點 (mm)
    double radius;       // 半徑 (mm)，-1 表示直線
    char direction;      // 'L', 'R', or '_'
    double angle_deg;    // 轉幾度到下一點
};

class CurvedPathGenerator : public rclcpp::Node {
public:
    CurvedPathGenerator() : Node("curved_path_generator") {
        pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&CurvedPathGenerator::publish_path, this));
        load_commands("/home/tdk/ros2_ws/src/navigation/waypoints/center_path.csv");
        build_path();
    }

private:
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    nav_msgs::msg::Path path_msg_;
    std::vector<PathCommand> commands_;

    void load_commands(const std::string &filename) {
        std::ifstream file(filename);
        std::string line;
        while (std::getline(file, line)) {
            if (line.empty() || line[0] == '#') continue;
            std::stringstream ss(line);
            std::string x_str, y_str, r_str, d_str, a_str;
            std::getline(ss, x_str, ',');
            std::getline(ss, y_str, ',');
            std::getline(ss, r_str, ',');
            std::getline(ss, d_str, ',');
            std::getline(ss, a_str, ',');
            commands_.push_back({
                std::stod(x_str),
                std::stod(y_str),
                std::stod(r_str),
                d_str.empty() ? '_' : d_str[0],
                std::stod(a_str)
            });
        }
    }

    void build_path() {
        path_msg_.header.frame_id = "map";
        if (commands_.size() < 2) return;

        // 初始位置與方向（向右）
        double x = commands_[0].x / 1000.0;
        double y = commands_[0].y / 1000.0;
        double yaw = 0.0;
        add_pose(x, y, yaw);

        for (size_t i = 1; i < commands_.size(); ++i) {
            const auto &cmd = commands_[i];
            double x1 = cmd.x / 1000.0;
            double y1 = cmd.y / 1000.0;

            if (cmd.radius < 0 || cmd.direction == '_') {
                interpolate_line(x, y, x1, y1, yaw);
                yaw = std::atan2(y1 - y, x1 - x);
                x = x1;
                y = y1;
            } else {
                interpolate_arc(x, y, yaw, cmd.radius / 1000.0, cmd.direction, cmd.angle_deg, x, y, yaw);
            }
        }
    }

    void interpolate_line(double x0, double y0, double x1, double y1, double yaw) {
        int steps = 30;
        for (int i = 1; i <= steps; ++i) {
            double t = static_cast<double>(i) / steps;
            double x = x0 + t * (x1 - x0);
            double y = y0 + t * (y1 - y0);
            add_pose(x, y, yaw);
        }
    }

    void interpolate_arc(double x0, double y0, double yaw_in, double radius,
                         char direction, double angle_deg,
                         double &x_out, double &y_out, double &yaw_out) {
        double sign = (direction == 'L') ? 1.0 : -1.0;
        double angle_rad = angle_deg * M_PI / 180.0 * sign;

        // 計算圓心
        double cx = x0 + radius * std::cos(yaw_in + sign * M_PI_2);
        double cy = y0 + radius * std::sin(yaw_in + sign * M_PI_2);

        // 起始角與結束角（從圓心看）
        double theta0 = std::atan2(y0 - cy, x0 - cx);
        double theta1 = theta0 + angle_rad;

        int steps = 30;
        for (int i = 1; i <= steps; ++i) {
            double t = static_cast<double>(i) / steps;
            double theta = theta0 + t * angle_rad;
            double x = cx + radius * std::cos(theta);
            double y = cy + radius * std::sin(theta);
            double tangent_yaw = theta + sign * M_PI_2;
            add_pose(x, y, tangent_yaw);
        }

        x_out = cx + radius * std::cos(theta1);
        y_out = cy + radius * std::sin(theta1);
        yaw_out = yaw_in + angle_rad;
    }

    void add_pose(double x, double y, double yaw) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.orientation.z = std::sin(yaw / 2.0);
        pose.pose.orientation.w = std::cos(yaw / 2.0);
        path_msg_.poses.push_back(pose);
    }

    void publish_path() {
        path_msg_.header.stamp = this->now();
        pub_->publish(path_msg_);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CurvedPathGenerator>());
    rclcpp::shutdown();
    return 0;
}