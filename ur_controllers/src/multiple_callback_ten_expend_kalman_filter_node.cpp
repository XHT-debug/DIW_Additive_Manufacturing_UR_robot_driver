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

rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr callback10_expend_kalman_filter_pub;

class MultipleCallbackTenExpendKalmanFilterSubscriber : public rclcpp::Node
{
public:
    MultipleCallbackTenExpendKalmanFilterSubscriber()
        : Node("expend_kalman_filter_ten")  // 设置节点名称
    {
        // 创建订阅者，订阅 "/expend_kalman_filter_result_k_1" 话题 
        expend_kalman_filter_result_k_1_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/expend_kalman_filter_result_k_1", 10,  // 话题名称和队列大小
            std::bind(&MultipleCallbackTenExpendKalmanFilterSubscriber::expend_kalman_filter_result_k_1_callback, this, std::placeholders::_1));

        callback10_expend_kalman_filter_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/callback10_expend_kalman_filter", 10);
    }

private:

    void expend_kalman_filter_result_k_1_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {

        std_msgs::msg::Float64MultiArray msg_send;
        msg_send.data.resize(19);
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(6, 1);
        Eigen::MatrixXd M_theta = Eigen::MatrixXd::Zero(6, 6);

        Eigen::MatrixXd theta = Eigen::MatrixXd::Zero(6, 1);
        Eigen::MatrixXd theta_perturb = Eigen::MatrixXd::Zero(6, 1);
        theta_perturb << msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[4], msg->data[5];
        theta = theta_perturb;
        Eigen::MatrixXd T_i_1_i_asb = joint_transform_matrix(theta);
        Eigen::MatrixXd dtheta = Eigen::MatrixXd::Zero(6, 1);
        dtheta << msg->data[6], msg->data[7], msg->data[8], msg->data[9], msg->data[10], msg->data[11];
        Eigen::MatrixXd tau = Eigen::MatrixXd::Zero(6, 1);
        tau << msg->data[12], msg->data[13], msg->data[14], msg->data[15], msg->data[16], msg->data[17];
        double dtheta_epsilon = 1e-6;

        // 计算惯性矩阵M(theta)
        for (int i = 1; i <= 6; i++) {
            Eigen::MatrixXd d2theta_i = Eigen::MatrixXd::Zero(6, 1);
            d2theta_i(i - 1, 0) = 1;
            M_theta.block<6, 1>(0, i - 1) = NewtonEulerInverseDynamics(theta, dtheta, d2theta_i, Eigen::MatrixXd::Zero(6, 1), false, T_i_1_i_asb);
        }

        for (int i = 1; i <= 3; i++) {
            Eigen::MatrixXd dtheta_perturb = dtheta;
            dtheta_perturb(i + 2) += dtheta_epsilon;
            // 计算科里奥利力矩,离心力矩,重力力矩 h(theta, dtheta)
            Eigen::MatrixXd h_theta_dtheta_perturb1 = NewtonEulerInverseDynamics(theta, dtheta_perturb, Eigen::MatrixXd::Zero(6, 1), Eigen::MatrixXd::Zero(6, 1), true, T_i_1_i_asb);
            dtheta_perturb(i + 2) -= 2 * dtheta_epsilon;
            Eigen::MatrixXd h_theta_dtheta_perturb2 = NewtonEulerInverseDynamics(theta, dtheta_perturb, Eigen::MatrixXd::Zero(6, 1), Eigen::MatrixXd::Zero(6, 1), true, T_i_1_i_asb);
            A = truncated_svd_solve(M_theta, h_theta_dtheta_perturb1 - h_theta_dtheta_perturb2) / (2 * dtheta_epsilon);
            msg_send.data[6 * (i - 1)] = A(0, 0);
            msg_send.data[6 * (i - 1) + 1] = A(1, 0);
            msg_send.data[6 * (i - 1) + 2] = A(2, 0);
            msg_send.data[6 * (i - 1) + 3] = A(3, 0);
            msg_send.data[6 * (i - 1) + 4] = A(4, 0);
            msg_send.data[6 * (i - 1) + 5] = A(5, 0);
        } 
        msg_send.data[18] = msg->data[18];
        callback10_expend_kalman_filter_pub->publish(msg_send);
    }

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr expend_kalman_filter_result_k_1_sub;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);  // 初始化 ROS 2
    auto node = std::make_shared<MultipleCallbackTenExpendKalmanFilterSubscriber>();  // 创建订阅者节点
    
    // 创建多线程执行器
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    
    // 使用多线程执行器运行节点
    executor.spin();  // 保持节点运行，监听消息
    
    rclcpp::shutdown();  // 关闭 ROS 2
    return 0;
}