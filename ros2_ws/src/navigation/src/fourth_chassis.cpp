#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int32.hpp>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <cstdlib>

struct PathCommand { double x, y, radius, angle; };

class CurvedPathPublisher : public rclcpp::Node {
public:
    CurvedPathPublisher() : Node("curved_path_publisher") {
        pub_twist = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        pub_end_ = this->create_publisher<std_msgs::msg::Int32>("mission_4_finish", 10);

        sub_start_ = this->create_subscription<std_msgs::msg::Int32>(
            "mission_4_start", 10,
            std::bind(&CurvedPathPublisher::startCallback, this, std::placeholders::_1)
        );

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
        timer_->cancel();  // 啟動時不啟動計時器，等收到開始訊號
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_twist;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_end_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_start_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<PathCommand> path_;
    size_t i_ = 1;
    int step_ = 0, max_step_ = 50;
    double prev_x_, prev_y_, prev_theta_;
    bool running_ = false;

    void startCallback(const std_msgs::msg::Int32::SharedPtr msg) {
        if (msg->data == 1 && !running_) {
            RCLCPP_INFO(this->get_logger(), "🚀 mission_4_start = 1, 開始跑路徑");
            running_ = true;
            i_ = 1;
            step_ = 0;

            prev_x_ = path_[0].x;
            prev_y_ = path_[0].y;

            if (path_.size() > 1) {
                if (std::abs(path_[0].angle) < 1e-3) {
                    // 第一段是直線 → 用直線方向初始化
                    prev_theta_ = std::atan2(
                        path_[1].y - path_[0].y,
                        path_[1].x - path_[0].x
                    );
                } else {
                    // 第一段是圓弧 → 用弧線切線方向初始化
                    double theta_rad = path_[0].angle * M_PI / 180.0;
                    double dx = path_[1].x - path_[0].x;
                    double dy = path_[1].y - path_[0].y;
                    double chord = std::hypot(dx, dy);
                    double r = std::abs(path_[0].radius);

                    double mid_x = (path_[0].x + path_[1].x) / 2.0;
                    double mid_y = (path_[0].y + path_[1].y) / 2.0;
                    double dir_x = -dy / chord;
                    double dir_y = dx / chord;
                    double h = std::sqrt(r * r - (chord * chord) / 4.0);
                    double cx = mid_x + dir_x * h * std::copysign(1.0, theta_rad);
                    double cy = mid_y + dir_y * h * std::copysign(1.0, theta_rad);

                    double start_angle = std::atan2(path_[0].y - cy, path_[0].x - cx);
                    double tangent_x = -std::sin(start_angle) * std::copysign(1.0, theta_rad);
                    double tangent_y =  std::cos(start_angle) * std::copysign(1.0, theta_rad);
                    prev_theta_ = std::atan2(tangent_y, tangent_x); // ✅ 弧線切線角度
                }
            } else {
                prev_theta_ = 0.0;
            }

            timer_->reset(); // ✅ 啟動 timer
        }
    }

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
        pub_twist->publish(twist);
    }

    void publishEndFlag(int value) {
        std_msgs::msg::Int32 msg;
        msg.data = value;
        pub_end_->publish(msg);
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
        // 已經跑完全部 path
        if (i_ >= path_.size()) {
            publishStop();
            publishEndFlag(1);
            timer_->cancel();
            running_ = false;
            return;
        } else {
            publishEndFlag(0);
        }

        const auto& p1 = path_[i_ - 1];
        const auto& p2 = path_[i_];
        double xi, yi, theta;

        // === step_ == 0 → 計算這段 max_step_ ===
        if (step_ == 0) {
            double path_length;
            if (std::abs(p1.angle) < 1e-3) {
                path_length = std::hypot(p2.x - p1.x, p2.y - p1.y);  // 直線距離 (cm)
            } else {
                path_length = std::abs(p1.radius) * std::abs(p1.angle) * M_PI / 180.0;  // 弧長 (cm)
            }

            double desired_speed = 50.0; // cm/s
            double dt = 0.05;            // 50ms
            max_step_ = std::ceil(path_length / (desired_speed * dt));
        }

        // === 計算當前 xi, yi, theta ===
        if (std::abs(p1.angle) < 1e-3) {
            // 直線
            double t = static_cast<double>(step_) / max_step_;
            xi = p1.x + (p2.x - p1.x) * t;
            yi = p1.y + (p2.y - p1.y) * t;
            theta = std::atan2(p2.y - p1.y, p2.x - p1.x);
        } else {
                // ===== 圓弧（含 ±180° 特判）=====
                const double theta_deg = p1.angle;
                const double theta_rad = theta_deg * M_PI / 180.0;
                const double r = std::abs(p1.radius);
                const double ANG_EPS = 1e-2;  // 角度接近判定
                const double EPS     = 1e-6;  // 浮點誤差

                double dx = p2.x - p1.x;
                double dy = p2.y - p1.y;
                double chord = std::hypot(dx, dy);

                // 幾何可行性：chord 不得明顯大於 2r
                if (chord > 2.0 * r + 1e-4) {
                    // 超出很多 → 無法構圓，退回直線內插
                    double t = static_cast<double>(step_) / max_step_;
                    xi = p1.x + dx * t;
                    yi = p1.y + dy * t;
                    theta = std::atan2(dy, dx);
                } else {
                    // 圓心 / 角度解卷
                    double mid_x = (p1.x + p2.x) * 0.5;
                    double mid_y = (p1.y + p2.y) * 0.5;
                    double dir_x = -dy / (chord + EPS);
                    double dir_y =  dx / (chord + EPS);

                    double cx, cy;
                    if (std::abs(std::abs(theta_deg) - 180.0) <= ANG_EPS) {
                        // ✅ 半圓特判：h=0，圓心即弦中點
                        cx = mid_x;
                        cy = mid_y;
                    } else {
                        // 一般圓：h = sqrt(r^2 - chord^2/4)（做 clamp 防負）
                        double inside = r*r - 0.25 * chord * chord;
                        if (inside < 0.0 && inside > -1e-6) inside = 0.0;
                        double h = std::sqrt(std::max(0.0, inside));
                        cx = mid_x + dir_x * h * std::copysign(1.0, theta_rad);
                        cy = mid_y + dir_y * h * std::copysign(1.0, theta_rad);
                    }

                    auto ang = [&](double X, double Y){ return std::atan2(Y - cy, X - cx); };
                    double a_start = ang(p1.x, p1.y);
                    double a_end   = ang(p2.x, p2.y);
                    if (theta_rad > 0 && a_end < a_start) a_end += 2*M_PI;
                    if (theta_rad < 0 && a_end > a_start) a_end -= 2*M_PI;

                    double t = static_cast<double>(step_) / max_step_;
                    double angle = a_start + (a_end - a_start) * t;

                    xi = cx + r * std::cos(angle);
                    yi = cy + r * std::sin(angle);

                    // 切線方向（與半徑垂直，正負由 angle 正負決定）
                    theta = angle + (theta_rad >= 0 ? +M_PI_2 : -M_PI_2);
                }
            }

        // 🚩 新增：避免新段落的第一個點速度跳成 0
        if (step_ == 0 && i_ > 1) {
            prev_x_ = xi;
            prev_y_ = yi;
            prev_theta_ = theta;
            step_++;
            return;   // 不輸出這筆，直接跳過
        }

        // === 用差分算速度 (cm/s) ===
        double dt = 0.05;
        double vx_cm = (xi - prev_x_) / dt; // xi, yi 單位 cm
        double vy_cm = (yi - prev_y_) / dt;
        double omega = (theta - prev_theta_) / dt; // rad/s

        double progress = static_cast<double>(step_) / max_step_;
        double factor = 1.0;

        // 第一段 → 只做加速
        if (i_ == 1) {
            if (progress < 0.2) {
                factor = progress / 0.2;   // 線性 0 → 1
            }
        }
        // 最後一段 → 只做減速
        else if (i_ == path_.size() - 1) {
            if (progress > 0.8) {
                factor = (1.0 - progress) / 0.2;  // 線性 1 → 0
            }
        }

        vx_cm *= factor;
        vy_cm *= factor;
        omega *= factor;

        // === cm/s → rps (下位機單位) ===
        double scale = 15.2 * M_PI;  // 你原本的換算比例
        double vx_rps = vx_cm / scale;
        double vy_rps = vy_cm / scale;

        geometry_msgs::msg::Twist twist;
        twist.linear.x = vx_rps;
        twist.linear.y = vy_rps;
        omega = 0;
        twist.angular.z = omega;
        pub_twist->publish(twist);

        // 更新狀態
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