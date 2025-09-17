#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <cstdlib>

struct PathCommand { double x, y, radius, angle; };

class CurvedPathPublisher : public rclcpp::Node {
public:
    CurvedPathPublisher() : Node("curved_path_publisher") {
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        path_ = readPath("src/navigation/waypoints/center_path.csv");
        if (path_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "❌ No valid path found.");
            return;
        }
        // ✅ 初始化 prev_x_, prev_y_, prev_theta_
        prev_x_ = path_[0].x;
        prev_y_ = path_[0].y;
        if (path_.size() > 1) {
            prev_theta_ = std::atan2(path_[1].y - path_[0].y, path_[1].x - path_[0].x);
        } else {
            prev_theta_ = 0.0;
        }

        i_ = 1;
        step_ = 0;
        timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&CurvedPathPublisher::publishTwist, this));
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<PathCommand> path_;
    size_t i_ = 1;
    int step_ = 0, max_step_ = 50;

    double prev_x_, prev_y_, prev_theta_;

    std::vector<PathCommand> readPath(const std::string& filename) {
        std::vector<PathCommand> path;
        std::ifstream file(filename);
        std::string line;

        while (std::getline(file, line)) {
            if (line.empty() || line[0] == '#') continue;
            std::stringstream ss(line);
            std::string x_str, y_str, r_str, _, a_str;

            std::getline(ss, x_str, ',');
            std::getline(ss, y_str, ',');
            std::getline(ss, r_str, ',');
            //std::getline(ss, _, ',');  // 跳過方向欄位
            std::getline(ss, a_str, ',');

            try {
                double x = std::stod(x_str);
                double y = std::stod(y_str);
                double r = std::stod(r_str);
                double a = std::stod(a_str);
                path.push_back({x, y, r, a});
            } catch (...) {
                RCLCPP_WARN(this->get_logger(), "⚠️ Invalid line: %s", line.c_str());
            }
        }
        return path;
    }

    void publishStop() {
        geometry_msgs::msg::Twist twist;
        twist.linear.x = twist.linear.y = twist.angular.z = 0.0;
        pub_->publish(twist);
    }

    double speedFactor(double progress) {
        if (progress < 0.2) {
            return 0.2 + 0.8 * (progress / 0.2);   // 前 30% 從 0.3 → 1.0
        } else if (progress > 0.8) {
            return 0.2 + 0.8 * ((1.0 - progress) / 0.2); // 後 30% 從 1.0 → 0.3
        }
        return 1.0;  // 中間維持 100%
    }

    void publishTwist() {
        // ✅ 啟動時先送一筆零速度，避免爆轉
        if (i_ == 1 && step_ == 0) {
            publishStop();
            step_++;   // 下一次再開始動
            return;
        }

        if (i_ >= path_.size()) {
            // 路徑走完 → 發送一次停止訊息
            publishStop();
            timer_->cancel();   // 停止計時器，不再發送
            return;
        }

        const auto& p1 = path_[i_ - 1];
        const auto& p2 = path_[i_];

        double xi, yi, theta;

        if (std::abs(p1.angle) < 1e-3) {
            double t = static_cast<double>(step_) / max_step_;
            xi = p1.x + (p2.x - p1.x) * t;
            yi = p1.y + (p2.y - p1.y) * t;
            theta = std::atan2(yi - prev_y_, xi - prev_x_);
        } else {
            double theta_rad = p1.angle * M_PI / 180.0;
            double dx = p2.x - p1.x;
            double dy = p2.y - p1.y;
            double chord = std::hypot(dx, dy);
            double r = std::abs(p1.radius);
            double mid_x = (p1.x + p2.x) / 2.0;
            double mid_y = (p1.y + p2.y) / 2.0;
            double dir_x = -dy / chord;
            double dir_y = dx / chord;
            double h = std::sqrt(r * r - (chord * chord) / 4.0);
            double cx = mid_x + dir_x * h * std::copysign(1.0, theta_rad);
            double cy = mid_y + dir_y * h * std::copysign(1.0, theta_rad);

            double start_angle = std::atan2(p1.y - cy, p1.x - cx);
            double end_angle = std::atan2(p2.y - cy, p2.x - cx);
            if (theta_rad > 0 && end_angle < start_angle) end_angle += 2 * M_PI;
            if (theta_rad < 0 && end_angle > start_angle) end_angle -= 2 * M_PI;
            double delta = end_angle - start_angle;
            double t = static_cast<double>(step_) / max_step_;
            double angle = start_angle + delta * t;
            xi = cx + r * std::cos(angle);
            yi = cy + r * std::sin(angle);
            theta = angle + M_PI_2 * std::copysign(1.0, theta_rad);
        }

        // 在 publishTwist() 中 step_ == 0 時設定 max_step_
        if (step_ == 0) {
            double path_length;
            double desired_speed;  // cm/s
            if (std::abs(p1.angle) < 1e-3) {
                path_length = std::hypot(p2.x - p1.x, p2.y - p1.y);  // 直線距離
                desired_speed = 15.0;  // cm/s
            } else {
                path_length = std::abs(p1.radius) * std::abs(p1.angle) * M_PI / 180.0;  // 弧長 = rθ
                desired_speed = 8.0;  // cm/s
            }

            double dt = 0.05;            // 控制間隔
            max_step_ = std::ceil(path_length / (desired_speed * dt));
        }

        double dt = 0.05;
        double vx = (xi - prev_x_) / dt;
        double vy = (yi - prev_y_) / dt;
        double omega = (theta - prev_theta_) / dt;
        double progress = static_cast<double>(step_) / max_step_;
        double factor = speedFactor(progress);

        vx *= factor;
        vy *= factor;
        omega *= factor;

        double scale = 15.2 * M_PI; // cm/s to rps
        vx /= scale;
        vy /= scale;

        geometry_msgs::msg::Twist twist;
        twist.linear.x = vx;
        twist.linear.y = vy;
        twist.angular.z = omega;
        pub_->publish(twist);

        prev_x_ = xi;
        prev_y_ = yi;
        prev_theta_ = theta;

        step_++;
        if (step_ > max_step_) {
            step_ = 0;
            i_++;
        }
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CurvedPathPublisher>());
    rclcpp::shutdown();
    return 0;
}