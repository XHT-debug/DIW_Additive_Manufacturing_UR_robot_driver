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

rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr callback11_expend_kalman_filter_pub;

class MultipleCallbackElevenExpendKalmanFilterSubscriber : public rclcpp::Node
{
public:
    MultipleCallbackElevenExpendKalmanFilterSubscriber()
        : Node("expend_kalman_filter_eleven")  // 设置节点名称
    {
        // 创建订阅者，订阅 "/expend_kalman_filter_result_k_1" 话题 
        expend_kalman_filter_result_k_1_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/expend_kalman_filter_result_k_1", 10,  // 话题名称和队列大小
            std::bind(&MultipleCallbackElevenExpendKalmanFilterSubscriber::expend_kalman_filter_result_k_1_callback, this, std::placeholders::_1));

        callback11_expend_kalman_filter_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/callback11_expend_kalman_filter", 10);
    }

private:

    void expend_kalman_filter_result_k_1_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {     
        
        Eigen::MatrixXd joint_positions = Eigen::MatrixXd::Zero(6, 1);
        joint_positions << msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[4], msg->data[5];
        Eigen::MatrixXd Z_Axis_observation_matrix = Z_Axis_observation_matrix_calculate(joint_positions);
        
        std_msgs::msg::Float64MultiArray msg_send;
        msg_send.data.resize(8);

        for (int i = 0; i < 6; i++) {
            msg_send.data[i] = Z_Axis_observation_matrix(i, 0);
        }
        msg_send.data[6] = Z_Axis_calculate(joint_positions);
        msg_send.data[7] = msg->data[18];

        callback11_expend_kalman_filter_pub->publish(msg_send);
        
    }

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr expend_kalman_filter_result_k_1_sub;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);  // 初始化 ROS 2
    auto node = std::make_shared<MultipleCallbackElevenExpendKalmanFilterSubscriber>();  // 创建订阅者节点
    
    // 创建多线程执行器
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    
    // 使用多线程执行器运行节点
    executor.spin();  // 保持节点运行，监听消息
    
    rclcpp::shutdown();  // 关闭 ROS 2
    return 0;
}