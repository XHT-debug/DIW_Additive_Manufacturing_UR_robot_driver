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
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr callback2_expend_kalman_filter_pub;
rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr send_update_callback_pub;

Eigen::MatrixXd A = Eigen::MatrixXd::Zero(12, 12);
double now_time = 0;
int result_3 = 0;
int result_4 = 0;
int result_5 = 0;
int result_6 = 0;
int result_7 = 0;
int result_8 = 0;
int result_9 = 0;
int result_10 = 0;
int result_11 = 0;
bool send_update = false;

class MultipleCallbackTwoExpendKalmanFilterSubscriber : public rclcpp::Node
{
public:
    MultipleCallbackTwoExpendKalmanFilterSubscriber()
        : Node("expend_kalman_filter_two")  // 设置节点名称
    {
        // 创建订阅者，订阅 "/joint_states" 话题
        send_update_sub = this->create_subscription<std_msgs::msg::Bool>(
            "/send_update", 10,  // 话题名称和队列大小
            std::bind(&MultipleCallbackTwoExpendKalmanFilterSubscriber::send_update_callback, this, std::placeholders::_1));

        // 创建订阅者，订阅 "/expend_kalman_filter_result_k_1" 话题 
        expend_kalman_filter_result_k_1_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/expend_kalman_filter_result_k_1", 10,  // 话题名称和队列大小
            std::bind(&MultipleCallbackTwoExpendKalmanFilterSubscriber::expend_kalman_filter_result_k_1_callback, this, std::placeholders::_1));

        // 创建订阅者，订阅 "/callback3_expend_kalman_filter" 话题 
        callback3_expend_kalman_filter_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/callback3_expend_kalman_filter", 10,  // 话题名称和队列大小
            std::bind(&MultipleCallbackTwoExpendKalmanFilterSubscriber::callback3_expend_kalman_filter_callback, this, std::placeholders::_1));

        // 创建订阅者，订阅 "/callback4_expend_kalman_filter" 话题 
        callback4_expend_kalman_filter_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/callback4_expend_kalman_filter", 10,  // 话题名称和队列大小
            std::bind(&MultipleCallbackTwoExpendKalmanFilterSubscriber::callback4_expend_kalman_filter_callback, this, std::placeholders::_1));

        // 创建订阅者，订阅 "/callback5_expend_kalman_filter" 话题 
        callback5_expend_kalman_filter_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/callback5_expend_kalman_filter", 10,  // 话题名称和队列大小
            std::bind(&MultipleCallbackTwoExpendKalmanFilterSubscriber::callback5_expend_kalman_filter_callback, this, std::placeholders::_1));

        // 创建订阅者，订阅 "/callback6_expend_kalman_filter" 话题 
        callback6_expend_kalman_filter_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/callback6_expend_kalman_filter", 10,  // 话题名称和队列大小
            std::bind(&MultipleCallbackTwoExpendKalmanFilterSubscriber::callback6_expend_kalman_filter_callback, this, std::placeholders::_1));

        // 创建订阅者，订阅 "/callback7_expend_kalman_filter" 话题 
        callback7_expend_kalman_filter_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/callback7_expend_kalman_filter", 10,  // 话题名称和队列大小
            std::bind(&MultipleCallbackTwoExpendKalmanFilterSubscriber::callback7_expend_kalman_filter_callback, this, std::placeholders::_1));
            
        // 创建订阅者，订阅 "/callback8_expend_kalman_filter" 话题 
        callback8_expend_kalman_filter_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/callback8_expend_kalman_filter", 10,  // 话题名称和队列大小
            std::bind(&MultipleCallbackTwoExpendKalmanFilterSubscriber::callback8_expend_kalman_filter_callback, this, std::placeholders::_1));

        // 创建订阅者，订阅 "/callback9_expend_kalman_filter" 话题       
        callback9_expend_kalman_filter_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/callback9_expend_kalman_filter", 10,  // 话题名称和队列大小
            std::bind(&MultipleCallbackTwoExpendKalmanFilterSubscriber::callback9_expend_kalman_filter_callback, this, std::placeholders::_1));

        // 创建订阅者，订阅 "/callback10_expend_kalman_filter" 话题 
        callback10_expend_kalman_filter_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>(  
            "/callback10_expend_kalman_filter", 10,  // 话题名称和队列大小
            std::bind(&MultipleCallbackTwoExpendKalmanFilterSubscriber::callback10_expend_kalman_filter_callback, this, std::placeholders::_1));

        callback11_expend_kalman_filter_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/callback11_expend_kalman_filter", 10,  // 话题名称和队列大小
            std::bind(&MultipleCallbackTwoExpendKalmanFilterSubscriber::callback11_expend_kalman_filter_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::nanoseconds(300),
            std::bind(&MultipleCallbackTwoExpendKalmanFilterSubscriber::timer_callback, this)
        );

        callback2_expend_kalman_filter_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/callback2_expend_kalman_filter", 10);
        send_update_callback_pub = this->create_publisher<std_msgs::msg::Bool>("/send_update_callback", 10);
    }

private:
    // 回调函数：处理 "/joint_state" 话题的消息
    void send_update_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        send_update = true;
        std_msgs::msg::Bool msg_send_update;
        msg_send_update.data = true;
        send_update_callback_pub->publish(msg_send_update);
    }

    void expend_kalman_filter_result_k_1_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        now_time = msg->data[18];
        // RCLCPP_INFO(this->get_logger(), "now_time = %f", now_time);
        bool whether_send = false;
        send_update = false;
        A = Eigen::MatrixXd::Zero(13, 12);
        result_3 = 0;
        result_4 = 0;
        result_5 = 0;
        result_6 = 0;
        result_7 = 0;
        result_8 = 0;
        result_9 = 0;
        result_10 = 0;
        result_11 = 0;
    }

    void callback3_expend_kalman_filter_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        A.block<6, 1>(6, 0) << msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[4], msg->data[5];
        if(msg->data[6] - now_time < 0.0001 && msg->data[6] - now_time > -0.0001) {
            result_3 = 1; 
        }
    }

    void callback4_expend_kalman_filter_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        A.block<6, 1>(6, 1) << msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[4], msg->data[5];
        if(msg->data[6] - now_time < 0.0001 && msg->data[6] - now_time > -0.0001) {
            result_4 = 1;
        }
    }

    void callback5_expend_kalman_filter_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        A.block<6, 1>(6, 2) << msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[4], msg->data[5];
        if(msg->data[6] - now_time < 0.0001 && msg->data[6] - now_time > -0.0001) {
            result_5 = 1;
        }
    }

    void callback6_expend_kalman_filter_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        A.block<6, 1>(6, 3) << msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[4], msg->data[5];
        if(msg->data[6] - now_time < 0.0001 && msg->data[6] - now_time > -0.0001) {
            result_6 = 1;
        }
    }

    void callback7_expend_kalman_filter_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        A.block<6, 1>(6, 4) << msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[4], msg->data[5];
        if(msg->data[6] - now_time < 0.0001 && msg->data[6] - now_time > -0.0001) {
            result_7 = 1;
        }
    }

    void callback8_expend_kalman_filter_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        A.block<6, 1>(6, 5) << msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[4], msg->data[5];
        if(msg->data[6] - now_time < 0.0001 && msg->data[6] - now_time > -0.0001) {
            result_8 = 1;
        }
    }

    void callback9_expend_kalman_filter_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        A.block<6, 1>(6, 6) << msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[4], msg->data[5];
        A.block<6, 1>(6, 7) << msg->data[6], msg->data[7], msg->data[8], msg->data[9], msg->data[10], msg->data[11];
        A.block<6, 1>(6, 8) << msg->data[12], msg->data[13], msg->data[14], msg->data[15], msg->data[16], msg->data[17];
        if(msg->data[18] - now_time < 0.0001 && msg->data[18] - now_time > -0.0001) {
            result_9 = 1;
        }
    }

    void callback10_expend_kalman_filter_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        A.block<6, 1>(6, 9) << msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[4], msg->data[5];
        A.block<6, 1>(6, 10) << msg->data[6], msg->data[7], msg->data[8], msg->data[9], msg->data[10], msg->data[11];
        A.block<6, 1>(6, 11) << msg->data[12], msg->data[13], msg->data[14], msg->data[15], msg->data[16], msg->data[17];
        if(msg->data[18] - now_time < 0.0001 && msg->data[18] - now_time > -0.0001) {
            result_10 = 1;
        }
    }

    void callback11_expend_kalman_filter_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        A.block<1, 7>(12, 0) << msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[4], msg->data[5], msg->data[6];
        if(msg->data[7] - now_time < 0.0001 && msg->data[7] - now_time > -0.0001) {
            result_11 = 1;
        }
    }

    void timer_callback() {
        bool whether_send = false;

        if (result_3 * result_4 * result_5 * result_6 * result_7 * result_8 * result_9 * result_10 * result_11 > 0) {
            whether_send = true;
            result_3 = 0;
            result_4 = 0;
            result_5 = 0;
            result_6 = 0;
            result_7 = 0;
            result_8 = 0;
            result_9 = 0;
            result_10 = 0;
            result_11 = 0;
        }

        if (whether_send) {
            std_msgs::msg::Float64MultiArray msg_send;
            msg_send.data.resize(80);
            for (int i = 1; i <= 6; i++) {
                msg_send.data[6*(i-1) + 0] = A(6, i-1);
                msg_send.data[6*(i-1) + 1] = A(7, i-1);
                msg_send.data[6*(i-1) + 2] = A(8, i-1);
                msg_send.data[6*(i-1) + 3] = A(9, i-1);
                msg_send.data[6*(i-1) + 4] = A(10, i-1);
                msg_send.data[6*(i-1) + 5] = A(11, i-1);
            }
            for (int i = 1; i <= 6; i++) {
                msg_send.data[36 + 6*(i-1) + 0] = A(6, i+5);
                msg_send.data[36 + 6*(i-1) + 1] = A(7, i+5);
                msg_send.data[36 + 6*(i-1) + 2] = A(8, i+5);
                msg_send.data[36 + 6*(i-1) + 3] = A(9, i+5);
                msg_send.data[36 + 6*(i-1) + 4] = A(10, i+5);
                msg_send.data[36 + 6*(i-1) + 5] = A(11, i+5);
            }
            for (int i = 1; i <= 7; i++) {
                msg_send.data[72 + i-1] = A(12, i-1);
            }
            msg_send.data[79] = now_time;
            callback2_expend_kalman_filter_pub->publish(msg_send);
        }
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr send_update_sub;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr expend_kalman_filter_result_k_1_sub;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr callback3_expend_kalman_filter_sub;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr callback4_expend_kalman_filter_sub;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr callback5_expend_kalman_filter_sub;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr callback6_expend_kalman_filter_sub;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr callback7_expend_kalman_filter_sub;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr callback8_expend_kalman_filter_sub;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr callback9_expend_kalman_filter_sub;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr callback10_expend_kalman_filter_sub;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr callback11_expend_kalman_filter_sub;
    rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);  // 初始化 ROS 2
    auto node = std::make_shared<MultipleCallbackTwoExpendKalmanFilterSubscriber>();  // 创建订阅者节点
    
    // 创建多线程执行器
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    
    // 使用多线程执行器运行节点
    executor.spin();  // 保持节点运行，监听消息
    
    rclcpp::shutdown();  // 关闭 ROS 2
    return 0;
}