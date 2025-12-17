#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>
#include <vector>
#include <string>

// Struttura dati per posizione 2D
struct Position2D {
    double x;
    double y;
};

// Struttura dati per pilastro/oggetto
struct Pillar {
    std::string name;
    Position2D pos;
};

class PickPlaceNode : public rclcpp::Node {
public:
    PickPlaceNode() : Node("pick_place_task") {
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        arm_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/position_controller/commands", 10);

        amcl_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/amcl_pose", 10,
            std::bind(&PickPlaceNode::amcl_callback, this, std::placeholders::_1)
        );

        pillars_ = {
            {"pillar_1", {2.0, 1.5}},
            {"pillar_2", {2.0, -1.5}},
            // ... altri pilastri ...
        };
        box_pos_ = {-2.0, 0.0};

        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&PickPlaceNode::main_loop, this));
        step_ = 0;
        pillar_idx_ = 0;
        robot_pose_x_ = 0.0;
        robot_pose_y_ = 0.0;
        robot_yaw_ = 0.0;
        RCLCPP_INFO(this->get_logger(), "PickPlaceNode avviato");
    }

private:
    void amcl_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        robot_pose_x_ = msg->pose.pose.position.x;
        robot_pose_y_ = msg->pose.pose.position.y;
        robot_yaw_ = tf2::getYaw(msg->pose.pose.orientation);
    }
    void main_loop() {
        if (pillar_idx_ >= pillars_.size()) {
            RCLCPP_INFO(this->get_logger(), "Task completato!");
            rclcpp::shutdown();
            return;
        }

        const auto &pillar = pillars_[pillar_idx_];
        double target_x = (step_ < 3) ? pillar.pos.x : box_pos_.x;
        double target_y = (step_ < 3) ? pillar.pos.y : box_pos_.y;
        double dist = std::hypot(robot_pose_x_ - target_x, robot_pose_y_ - target_y);
        double soglia_arrivo = 0.3; // metri

        switch (step_) {
            case 0:
                RCLCPP_INFO(this->get_logger(), "Navigazione verso %s (x=%.2f, y=%.2f)", pillar.name.c_str(), pillar.pos.x, pillar.pos.y);
                if (dist > soglia_arrivo) {
                    publish_cmd_vel(0.2, 0.2);
                } else {
                    publish_cmd_vel(0.0, 0.0);
                    step_++;
                }
                break;
            case 1:
                if (dist <= soglia_arrivo) {
                    RCLCPP_INFO(this->get_logger(), "Arrivato davanti al pilastro");
                    publish_cmd_vel(0.0, 0.0);
                    step_++;
                } else {
                    publish_cmd_vel(0.2, 0.2);
                }
                break;
            case 2:
                RCLCPP_INFO(this->get_logger(), "Controllo braccio per pick (simulato)");
                publish_arm({1.0, 0.5, -1.0, -0.5, 0.0});
                step_++;
                break;
            case 3:
                if (dist > soglia_arrivo) {
                    RCLCPP_INFO(this->get_logger(), "Navigazione verso box (x=%.2f, y=%.2f)", box_pos_.x, box_pos_.y);
                    publish_cmd_vel(0.2, 0.0);
                } else {
                    publish_cmd_vel(0.0, 0.0);
                    step_++;
                }
                break;
            case 4:
                if (dist <= soglia_arrivo) {
                    RCLCPP_INFO(this->get_logger(), "Rilascio oggetto nella box (simulato)");
                    publish_arm({0.0, 0.0, 0.0, 0.0, 0.06});
                    step_ = 0;
                    pillar_idx_++;
                } else {
                    publish_cmd_vel(0.2, 0.0);
                }
                break;
        }
    }

    void publish_cmd_vel(double linear_x, double angular_z) {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = linear_x;
        msg.angular.z = angular_z;
        cmd_vel_pub_->publish(msg);
    }

    void publish_arm(const std::vector<double> &joints) {
        auto msg = std_msgs::msg::Float64MultiArray();
        msg.data = joints;
        arm_pub_->publish(msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr arm_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
    std::vector<Pillar> pillars_;
    Position2D box_pos_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t step_;
    size_t pillar_idx_;
    double robot_pose_x_;
    double robot_pose_y_;
    double robot_yaw_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PickPlaceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}