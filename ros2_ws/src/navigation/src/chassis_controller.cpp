#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/int32.hpp>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <utility>

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
        sub_odom_ = this->create_subscription<geometry_msgs::msg::Point>(
            "wheel_odom", 10,
            std::bind(&CurvedPathPublisher::odomCallback, this, std::placeholders::_1)
        );

        path_ = readPath("src/navigation/waypoints/center_path.csv");
        if (path_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "❌ No valid path found.");
            return;
        }

        // ✅ 初始化 prev_x_, prev_y_
        prev_x_ = path_[0].x;
        prev_y_ = path_[0].y;

        i_ = 1;
        step_ = 0;
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&CurvedPathPublisher::publishTwist, this)
        );
        timer_->cancel();  // 啟動時不啟動計時器，等收到開始訊號
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_twist;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_end_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_start_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_odom_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<PathCommand> path_;
    size_t i_ = 1;
    int step_ = 0, max_step_ = 50;
    double prev_x_, prev_y_;
    bool running_ = false;

    bool have_odom_ = false;
    double x_real_ = 0.0, y_real_ = 0.0;
    double tolerance_ = 5.0; // cm
    double Kp_ = 0.05;       // 誤差修正增益
    int warmup_ticks_ = 0;          // 段切換後的助跑倒數
    const int WARMUP_TICKS_MAX = 6; // 6*50ms = 300ms 助跑時間

    std::pair<double,double> projectToSegment_
    (double px, double py, double x1, double y1,
    double x2, double y2, double* s_out = nullptr) {
        const double vx = x2 - x1, vy = y2 - y1;
        const double L2 = vx*vx + vy*vy + 1e-9;
        double s = ((px - x1)*vx + (py - y1)*vy) / L2; // 無夾
        if (s < 0.0) s = 0.0;
        if (s > 1.0) s = 1.0;
        if (s_out) *s_out = s;
        return { x1 + s*vx, y1 + s*vy };
    }

    void startCallback(const std_msgs::msg::Int32::SharedPtr msg) {
        if (msg->data == 1 && !running_) {
            RCLCPP_INFO(this->get_logger(), "🚀 mission_4_start = 1, 開始跑路徑");
            running_ = true;
            i_ = 1;
            step_ = 0;

            prev_x_ = path_[0].x;
            prev_y_ = path_[0].y;

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
            std::string x_str, y_str, r_str, a_str;

            std::getline(ss, x_str, ',');
            std::getline(ss, y_str, ',');
            std::getline(ss, r_str, ',');
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
        twist.linear.x = twist.linear.y = 0.0;
        twist.angular.z = 0.0;
        pub_twist->publish(twist);
    }

    void publishEndFlag(int value) {
        std_msgs::msg::Int32 msg;
        msg.data = value;
        pub_end_->publish(msg);
    }

    void odomCallback(const geometry_msgs::msg::Point::SharedPtr msg) {
        x_real_ = msg->x; // m → cm
        y_real_ = msg->y; // m → cm
        x_real_ *= -100.0;
        y_real_ *= 100.0;
        have_odom_ = true;
    }

void publishTwist() {
    // 1) 還沒開始或沒有里程就不動
    if (!running_ || !have_odom_) {
        publishStop();
        return;
    }

    // 2) 全部路徑完成
    if (i_ >= path_.size()) {
        publishStop();
        publishEndFlag(1);
        timer_->cancel();
        running_ = false;
        return;
    } else {
        publishEndFlag(0);
    }

    // ===== 參數（可依車況微調）=====
    const double dt          = 0.05;   // 50 ms
    const double vmax_ms     = 0.7;   // 上限 m/s
    const double vmax_cms    = vmax_ms * 100.0;
    const double Kp_track    = 0.12;   // 拉回 P（cm/s per cm）
    const double Kd_track    = 0.05;   // 阻尼 D（cm/s per cm/s）
    const double deadband    = 2.0;    // <1cm 改縮放，不是歸零
    const double ff_speed    = 50.0;   // 沿路徑底速 (cm/s)
    const double vff_min     = 30.0;    // 沿切線最小前饋地板 (cm/s)

    const double snap_radius = 10.0;   // 距終點 <10cm → 目標直接貼終點
    const double goal_tol    = 1.0;    // 必須 <1cm 才算到點
    const double lookahead   = 20.0;   // 段切換助跑前視距離（cm）←原10
    // 將成員 const int WARMUP_TICKS_MAX 建議設 10（原 6）

    auto clamp = [](double v, double lim){
        return (v >  lim) ?  lim : (v < -lim ? -lim : v);
    };

    const auto& p1 = path_[i_ - 1];
    const auto& p2 = path_[i_];

    // 3) 接近終點先 snap
    double dist_to_p2 = std::hypot(p2.x - x_real_, p2.y - y_real_);
    bool   near_goal  = (dist_to_p2 <= snap_radius);

    // 4) 目標點 (xt, yt) 與切線方向 (tnx, tny)
    double xt = 0.0, yt = 0.0;     // 追蹤目標
    double tnx = 0.0, tny = 0.0;   // 切線單位向量（前饋方向）

    if (near_goal) {
        // ---- 貼點：直接追 waypoint ----
        xt = p2.x; 
        yt = p2.y;
        double gx = xt - x_real_, gy = yt - y_real_;
        double gL = std::hypot(gx, gy);
        tnx = (gL > 1e-6) ? gx / gL : 0.0;
        tny = (gL > 1e-6) ? gy / gL : 0.0;
    } else {
        // 直線段判定：半徑近 0 或角度近 0 視為直線
        bool isArc = (std::abs(p1.radius) > 1e-6) && (std::abs(p1.angle) > 1e-3);

        if (!isArc) {
            // ---- 直線段：投影 + blend 到末端，切線為線段方向 ----
            double dx = p2.x - p1.x, dy = p2.y - p1.y;
            double tL = std::hypot(dx, dy);
            tnx = (tL > 1e-6) ? dx / tL : 0.0;
            tny = (tL > 1e-6) ? dy / tL : 0.0;

            double vx = dx, vy = dy;
            double L2 = vx*vx + vy*vy + 1e-9;
            double s = ((x_real_ - p1.x)*vx + (y_real_ - p1.y)*vy) / L2;
            if (s < 0.0) s = 0.0;
            if (s > 1.0) s = 1.0;
            double xs = p1.x + s*vx, ys = p1.y + s*vy;

            if (warmup_ticks_ > 0) {
                // 段切換助跑：先往切線方向看一小段
                xt = p1.x + tnx * lookahead;
                yt = p1.y + tny * lookahead;
            } else {
                // 接近尾端做平滑銜接
                double blend = (s > 0.8) ? (s - 0.8) / 0.2 : 0.0;
                if (blend > 1.0) blend = 1.0;
                xt = (1.0 - blend) * xs + blend * p2.x;
                yt = (1.0 - blend) * ys + blend * p2.y;
            }
        } else {
            // ---- 弧線段：依半徑與角度求圓心，沿弧前推 ----
            const double theta_deg = p1.angle;
            const double theta_rad = theta_deg * M_PI / 180.0;
            const double r = std::abs(p1.radius);
            const double ANG_EPS = 1e-2;
            const double EPS     = 1e-6;

            double dx = p2.x - p1.x, dy = p2.y - p1.y;
            double chord = std::hypot(dx, dy);

            // chord > 2r（不可能弧）→ 退回直線處理
            if (chord > 2.0 * r + 1e-4) {
                double tL = std::hypot(dx, dy);
                tnx = (tL > 1e-6) ? dx / tL : 0.0;
                tny = (tL > 1e-6) ? dy / tL : 0.0;

                double vx = dx, vy = dy;
                double L2 = vx*vx + vy*vy + 1e-9;
                double s = ((x_real_ - p1.x)*vx + (y_real_ - p1.y)*vy) / L2;
                if (s < 0.0) s = 0.0;
                if (s > 1.0) s = 1.0;
                double xs = p1.x + s*vx, ys = p1.y + s*vy;

                if (warmup_ticks_ > 0) {
                    xt = p1.x + tnx * lookahead;
                    yt = p1.y + tny * lookahead;
                } else {
                    double blend = (s > 0.8) ? (s - 0.8) / 0.2 : 0.0;
                    if (blend > 1.0) blend = 1.0;
                    xt = (1.0 - blend) * xs + blend * p2.x;
                    yt = (1.0 - blend) * ys + blend * p2.y;
                }
            } else {
                // 求圓心
                double mid_x = (p1.x + p2.x) * 0.5;
                double mid_y = (p1.y + p2.y) * 0.5;
                double dir_x = -dy / (chord + EPS);
                double dir_y =  dx / (chord + EPS);

                double cx, cy;
                if (std::abs(std::abs(theta_deg) - 180.0) <= ANG_EPS) {
                    // 半圓：圓心=弦中點
                    cx = mid_x; cy = mid_y;
                } else {
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

                // 把目前位置限制在弧段範圍，再沿弧前推一步
                double a_cur = ang(x_real_, y_real_);
                while (theta_rad > 0 && a_cur < a_start) a_cur += 2*M_PI;
                while (theta_rad > 0 && a_cur > a_end)   a_cur -= 2*M_PI;
                while (theta_rad < 0 && a_cur > a_start) a_cur -= 2*M_PI;
                while (theta_rad < 0 && a_cur < a_end)   a_cur += 2*M_PI;

                double a_lo = std::min(a_start, a_end);
                double a_hi = std::max(a_start, a_end);
                double a_proj = (a_cur < a_lo) ? a_lo : (a_cur > a_hi ? a_hi : a_cur);

                // 底速轉角速度：dθ = v/r * dt
                double dAng = (ff_speed / r) * dt * ((theta_rad >= 0) ? +1.0 : -1.0);
                double a_tgt = a_proj + dAng;
                if (theta_rad > 0) a_tgt = std::min(a_tgt, a_end);
                else               a_tgt = std::max(a_tgt, a_end);

                xt = cx + r * std::cos(a_tgt);
                yt = cy + r * std::sin(a_tgt);

                // 切線方向（與半徑垂直）
                double rx = xt - cx, ry = yt - cy;
                double tx = (theta_rad >= 0) ? -ry :  ry;
                double ty = (theta_rad >= 0) ?  rx : -rx;
                double tL = std::hypot(tx, ty);
                tnx = (tL > 1e-6) ? tx / tL : 0.0;
                tny = (tL > 1e-6) ? ty / tL : 0.0;

                // 段切換助跑
                if (warmup_ticks_ > 0) {
                    xt = p1.x + tnx * lookahead;
                    yt = p1.y + tny * lookahead;
                }
            }
        }
    }

    // 5) 誤差（cm）與微分（cm/s）
    static double ex_prev = 0.0, ey_prev = 0.0;
    double ex = xt - x_real_, ey = yt - y_real_;
    double dex = (ex - ex_prev) / dt;
    double dey = (ey - ey_prev) / dt;
    ex_prev = ex; ey_prev = ey;

    // 6) 控制律：沿切線前饋 + PD 拉回（cm/s）
    double vx_cmd = ff_speed * tnx + Kp_track * ex - Kd_track * dex;
    double vy_cmd = ff_speed * tny + Kp_track * ey - Kd_track * dey;

    // 半死區（縮放）與沿切線最小前饋
    double err = std::hypot(ex, ey);
    if (err < deadband) {
        double scale = err / deadband; // 0~1
        vx_cmd *= scale;
        vy_cmd *= scale;
    }
    // 沿切線的最小前饋（除非已經非常接近目標）
    double v_along = vx_cmd * tnx + vy_cmd * tny; // 投影到切線
    if (std::abs(v_along) < vff_min && dist_to_p2 > goal_tol) {
        double boost = (v_along >= 0 ? (vff_min - v_along) : -(vff_min + v_along));
        vx_cmd += boost * tnx;
        vy_cmd += boost * tny;
    }

    // 飽和
    vx_cmd = clamp(vx_cmd, vmax_cms);
    vy_cmd = clamp(vy_cmd, vmax_cms);

    // 7) 到點判定：一定走到點才切段
    dist_to_p2 = std::hypot(p2.x - x_real_, p2.y - y_real_);
    if (dist_to_p2 <= goal_tol) {
        // 進到下一段
        i_++;
        if (i_ >= path_.size()) {
            publishStop();
            publishEndFlag(1);
            timer_->cancel();
            running_ = false;
            return;
        }
        // 換段：清誤差、啟動助跑，不剎停
        ex_prev = ey_prev = 0.0;
        warmup_ticks_ = WARMUP_TICKS_MAX;

        // >>> 立刻推一腳（kick）：朝下一個 p2 給一個小速度，避免停住
        const auto& next_p2 = path_[i_];
        double gx = next_p2.x - x_real_;
        double gy = next_p2.y - y_real_;
        double gL = std::hypot(gx, gy);
        if (gL > 1e-6) { gx /= gL; gy /= gL; }
        geometry_msgs::msg::Twist kick;
        kick.linear.x = (std::max(vff_min, 20.0) / 100.0) * gx; // 12 cm/s 起步
        kick.linear.y = (std::max(vff_min, 20.0) / 100.0) * gy;
        kick.angular.z = 0.0;
        pub_twist->publish(kick);
        return; // 本 tick 結束，下一 tick 依新段正常計算
    }

    // 8) 發送（cm/s → m/s）
    geometry_msgs::msg::Twist twist;
    twist.linear.x = vx_cmd / 100.0;
    twist.linear.y = vy_cmd / 100.0;
    twist.angular.z = 0.0;
    pub_twist->publish(twist);

    // 助跑倒數
    if (warmup_ticks_ > 0) warmup_ticks_--;
}

};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CurvedPathPublisher>());
    rclcpp::shutdown();
    return 0;
}