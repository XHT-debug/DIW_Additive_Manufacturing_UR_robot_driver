#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <iostream>
#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include <chrono>
#include <random>
#include "ur_controllers/dynamics_statespace.hpp"

#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr callback6_expend_kalman_filter_pub;

class MultipleCallbackSixExpendKalmanFilterSubscriber : public rclcpp::Node
{
public:
    MultipleCallbackSixExpendKalmanFilterSubscriber()
        : Node("expend_kalman_filter_six")  // 设置节点名称
    {
        // 创建订阅者，订阅 "/expend_kalman_filter_result_k_1" 话题 
        expend_kalman_filter_result_k_1_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/expend_kalman_filter_result_k_1", 10,  // 话题名称和队列大小
            std::bind(&MultipleCallbackSixExpendKalmanFilterSubscriber::expend_kalman_filter_result_k_1_callback, this, std::placeholders::_1));

        callback6_expend_kalman_filter_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/callback6_expend_kalman_filter", 10);
    }

private:

    void expend_kalman_filter_result_k_1_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        
        Eigen::MatrixXd theta_perturb = Eigen::MatrixXd::Zero(6, 1);
        theta_perturb << msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[4], msg->data[5];
        Eigen::MatrixXd dtheta = Eigen::MatrixXd::Zero(6, 1);
        dtheta << msg->data[6], msg->data[7], msg->data[8], msg->data[9], msg->data[10], msg->data[11];
        Eigen::MatrixXd tau = Eigen::MatrixXd::Zero(6, 1);
        tau << msg->data[12], msg->data[13], msg->data[14], msg->data[15], msg->data[16], msg->data[17];
        double theta_epsilon = 1e-6;
        
        theta_perturb(3, 0) += theta_epsilon;
        Eigen::MatrixXd d2theta_perturb1 = NewtonEulerForwardDynamics(theta_perturb, dtheta, tau);
        theta_perturb(3, 0) -= 2 * theta_epsilon;
        Eigen::MatrixXd d2theta_perturb2 = NewtonEulerForwardDynamics(theta_perturb, dtheta, tau);

        std_msgs::msg::Float64MultiArray msg_send;
        msg_send.data.resize(7);

        Eigen::MatrixXd A_part = (d2theta_perturb1 - d2theta_perturb2) / (2 * theta_epsilon);
        msg_send.data[0] = A_part(0, 0);
        msg_send.data[1] = A_part(1, 0);
        msg_send.data[2] = A_part(2, 0);
        msg_send.data[3] = A_part(3, 0);
        msg_send.data[4] = A_part(4, 0);
        msg_send.data[5] = A_part(5, 0);
        msg_send.data[6] = msg->data[18];

        callback6_expend_kalman_filter_pub->publish(msg_send);
    }

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr expend_kalman_filter_result_k_1_sub;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);  // 初始化 ROS 2
    auto node = std::make_shared<MultipleCallbackSixExpendKalmanFilterSubscriber>();  // 创建订阅者节点
    
    // 创建多线程执行器
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    
    // 使用多线程执行器运行节点
    executor.spin();  // 保持节点运行，监听消息
    
    rclcpp::shutdown();  // 关闭 ROS 2
    return 0;
}