#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <iostream>
#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include <chrono>
#include <random>
#include "ur_controllers/dynamics_statespace.hpp"

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <builtin_interfaces/msg/time.hpp>


rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr theta_from_joint_position_pub;
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr theta_from_joint_position_with_noise_pub;
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr theta_from_expend_kalman_filter_pub;
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr theta_velocity_from_joint_velocity_pub;
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr effort_from_joint_efforts_with_noise_pub;
rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr expend_kalman_filter_result_k_1_pub;
rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr z_axis_error_pub;
rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr send_update_pub;
expend_kalman_filter_result expend_kalman_filter_result_k;
expend_kalman_filter_result expend_kalman_filter_result_k_1;
bool start_expend_kalman_filter = false;
bool first_time = true;
bool send_update_callback = false;
Eigen::MatrixXd theta_k_outside = Eigen::MatrixXd::Zero(6, 1);
Eigen::MatrixXd theta_velocity_k_outside = Eigen::MatrixXd::Zero(6, 1);
double dt_outside = 0.008; // 默认值
Eigen::MatrixXd torque_k_1 = Eigen::MatrixXd::Zero(6, 1);
Eigen::MatrixXd X_k_prior = Eigen::MatrixXd::Zero(12, 1); //先验状态估计
Eigen::MatrixXd A_callback = Eigen::MatrixXd::Zero(12, 12);
Eigen::MatrixXd Ad_callback = Eigen::MatrixXd::Zero(12, 12);
Eigen::MatrixXd Z_Axis_callback = Eigen::MatrixXd::Zero(1, 7);
bool callback2_expend_kalman_filter_result = false;
bool start_next_expend_kalman_filter = true;

// 用于校正真实机械臂的力矩
Eigen::MatrixXd joint_positions_correction = Eigen::MatrixXd::Zero(6, 1);
Eigen::MatrixXd joint_velocitys_correction = Eigen::MatrixXd::Zero(6, 1);
Eigen::MatrixXd joint_efforts_correction = Eigen::MatrixXd::Zero(6, 1);
Eigen::MatrixXd effort_correction = Eigen::MatrixXd::Zero(6, 1);
bool start_torque_correction = false;
bool whether_torque_correction = false;
int number_of_correction = 0;

// 用于Z轴计算距离
double realtime_z_from_laser = 0;
double Z_Axis_zero_position = 0.272;
int laser_data = 0;

void calculateExpendKalmanFilter_prior(const Eigen::MatrixXd& torque_k_1, const double& dt)
{
    // 计算先验状态估计
    X_k_prior.block<6, 1>(0, 0) = expend_kalman_filter_result_k_1.X_posterior.block<6, 1>(0, 0) + expend_kalman_filter_result_k_1.X_posterior.block<6, 1>(6, 0) * dt;
    // X_k_prior.block<6, 1>(0, 0) = expend_kalman_filter_result_k_1.X_posterior.block<6, 1>(0, 0) + dt * theta_velocity_k_outside;
    X_k_prior.block<6, 1>(6, 0) = expend_kalman_filter_result_k_1.X_posterior.block<6, 1>(6, 0) + NewtonEulerForwardDynamics(expend_kalman_filter_result_k_1.X_posterior.block<6, 1>(0, 0), expend_kalman_filter_result_k_1.X_posterior.block<6, 1>(6, 0), torque_k_1) * dt;
}

void calculateExpendKalmanFilter_posterior(const Eigen::MatrixXd& Ad, const double& dt)
{
    Eigen::MatrixXd X_k_posterior = Eigen::MatrixXd::Zero(12, 1); //后验状态估计
    Eigen::MatrixXd P_k_prior = Eigen::MatrixXd::Zero(12, 12); // 先验估计协方差矩阵
    Eigen::MatrixXd P_k_posterior = Eigen::MatrixXd::Zero(12, 12); // 后验估计协方差矩阵
    Eigen::MatrixXd K_k = Eigen::MatrixXd::Zero(12, 6); // 卡尔曼增益矩阵
    Eigen::MatrixXd H_k = Eigen::MatrixXd::Zero(13, 12); // 观测矩阵
    Eigen::MatrixXd R_k = Eigen::MatrixXd::Zero(13, 13); // 观测噪声协方差矩阵
    Eigen::MatrixXd Q_k = Eigen::MatrixXd::Zero(12, 12); // 过程噪声协方差矩阵


    R_k.block<6, 6>(0, 0) << 0.00002234 * 0.00002234, 0, 0, 0, 0, 0,
                             0, 0.00002192 * 0.00002192, 0, 0, 0, 0,
                             0, 0, 0.00002274 * 0.00002274, 0, 0, 0,
                             0, 0, 0, 0.0000184 * 0.0000184, 0, 0,
                             0, 0, 0, 0, 0.000019 * 0.000019, 0,
                             0, 0, 0, 0, 0, 0.00002184 * 0.00002184;

    R_k.block<6, 6>(6, 6) << 0.000002 * 0.000002, 0, 0, 0, 0, 0,
                             0, 0.000002 * 0.000002, 0, 0, 0, 0,
                             0, 0, 0.000002 * 0.000002, 0, 0, 0,
                             0, 0, 0, 0.000002 * 0.000002, 0, 0,
                             0, 0, 0, 0, 0.000002 * 0.000002, 0,
                             0, 0, 0, 0, 0, 0.000002 * 0.000002;
                             
    R_k.block<1, 1>(12, 12) << 0.00000001 * 0.00000001;


    // 过程噪声协方差矩阵
    Q_k.block<6, 6>(0, 0) << 0.00002234 * 0.00002234, 0, 0, 0, 0, 0,
                             0, 0.00002192 * 0.00002192, 0, 0, 0, 0,
                             0, 0, 0.00002274 * 0.00002274, 0, 0, 0,
                             0, 0, 0, 0.0000184 * 0.0000184, 0, 0,
                             0, 0, 0, 0, 0.000019 * 0.000019, 0,
                             0, 0, 0, 0, 0, 0.00002184 * 0.00002184;

    Q_k.block<6, 6>(6, 6) << 0.0025 * 0.0025, 0, 0, 0, 0, 0,
                             0, 0.0025 * 0.0025, 0, 0, 0, 0,
                             0, 0, 0.0025 * 0.0025, 0, 0, 0,
                             0, 0, 0, 0.0025 * 0.0025, 0, 0,
                             0, 0, 0, 0, 0.0025 * 0.0025, 0,
                             0, 0, 0, 0, 0, 0.0025 * 0.0025;

    // 修改
    Q_k = 20 * Q_k;
    R_k = 1 * R_k;

    H_k.block<6, 6>(0, 0) = Eigen::MatrixXd::Identity(6, 6);
    H_k.block<6, 6>(6, 6) = Eigen::MatrixXd::Identity(6, 6);
    H_k.block<1, 6>(12, 0) = Z_Axis_callback.block<1, 6>(0, 0);

    // 计算先验估计协方差矩阵P_k_prior
    P_k_prior = Ad * expend_kalman_filter_result_k_1.P_posterior * Ad.transpose() + Q_k;

    // 计算卡尔曼增益矩阵
    Eigen::MatrixXd S = H_k * P_k_prior * H_k.transpose() + R_k;
    
    // 检查矩阵是否可逆
    Eigen::FullPivLU<Eigen::MatrixXd> lu(S);
    if (lu.isInvertible()) {
        K_k = P_k_prior * H_k.transpose() * S.inverse();
    } else {
        // 如果矩阵不可逆，使用伪逆或设置增益为零
        // RCLCPP_WARN(rclcpp::get_logger("expend_kalman_filter"), "Measurement covariance matrix is singular, using pseudo-inverse");
        K_k = P_k_prior * H_k.transpose() * S.completeOrthogonalDecomposition().pseudoInverse();
    }

    // Eigen::MatrixXd H_k_theta_k = H_k * X_k_prior;
    Eigen::MatrixXd data_k_outside = Eigen::MatrixXd::Zero(13, 1);
    data_k_outside.block<6, 1>(0, 0) = theta_k_outside;
    data_k_outside.block<6, 1>(6, 0) = theta_velocity_k_outside;
    data_k_outside(12, 0) = realtime_z_from_laser;

    Eigen::MatrixXd data_estimated = Eigen::MatrixXd::Zero(13, 1);
    data_estimated.block<12, 1>(0, 0) = X_k_prior;
    data_estimated(12, 0) = Z_Axis_callback(0, 6);

    std_msgs::msg::Float64 z_axis_error;
    z_axis_error.data = data_k_outside(12, 0) - data_estimated(12, 0);
    z_axis_error_pub->publish(z_axis_error);
    
    // 计算后验状态估计
    X_k_posterior = X_k_prior + K_k * (data_k_outside - data_estimated);
    
    // 计算后验估计协方差矩阵P_k_posterior
    P_k_posterior = (Eigen::MatrixXd::Identity(12, 12) - K_k * H_k) * P_k_prior;

    expend_kalman_filter_result_k.X_posterior = X_k_posterior;
    expend_kalman_filter_result_k.P_posterior = P_k_posterior;
}

double gaussian_perturbation(double standard_deviation) {
    // 使用 static 保证引擎和分布只初始化一次，避免重复播种
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::normal_distribution<double> dist(0.0, standard_deviation);  // 均值0，标准差

    return dist(gen);
}

class MultipleCallbackOneExpendKalmanFilterSubscriber : public rclcpp::Node
{
public:
    MultipleCallbackOneExpendKalmanFilterSubscriber()
        : Node("expend_kalman_filter_one")  // 设置节点名称
    {
        // 创建订阅者，订阅 "/joint_states" 话题
        joint_states_sub = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,  // 话题名称和队列大小
            std::bind(&MultipleCallbackOneExpendKalmanFilterSubscriber::joint_state_callback, this, std::placeholders::_1));

        // 创建订阅者，订阅 "/start_expend_kalman_filter" 话题
        start_expend_kalman_filter_sub = this->create_subscription<std_msgs::msg::Bool>(
            "/start_expend_kalman_filter", 10,  // 话题名称和队列大小
            std::bind(&MultipleCallbackOneExpendKalmanFilterSubscriber::start_expend_kalman_filter_callback, this, std::placeholders::_1));

        // 创建订阅者，订阅 "/start_debug" 话题 
        start_debug_sub = this->create_subscription<std_msgs::msg::Bool>(
            "/start_debug", 10,  // 话题名称和队列大小
            std::bind(&MultipleCallbackOneExpendKalmanFilterSubscriber::start_debug_callback, this, std::placeholders::_1));

        // 创建订阅者，订阅 "/callback2_expend_kalman_filter" 话题 
        callback2_expend_kalman_filter_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/callback2_expend_kalman_filter", 10,  // 话题名称和队列大小
            std::bind(&MultipleCallbackOneExpendKalmanFilterSubscriber::callback2_expend_kalman_filter_callback, this, std::placeholders::_1));

        // 创建订阅者，订阅 "/send_update_callback" 话题
        send_update_callback_sub = this->create_subscription<std_msgs::msg::Bool>(
            "/send_update_callback", 10,  // 话题名称和队列大小
            std::bind(&MultipleCallbackOneExpendKalmanFilterSubscriber::send_update_callback_callback, this, std::placeholders::_1));

        // 创建订阅者，订阅 "/realtime_distance_left" 话题
        realtime_distance_left_sub = this->create_subscription<std_msgs::msg::Int32>(
            "/realtime_distance_left",  // topic name
            10,  // QoS profile - queue size
            std::bind(&MultipleCallbackOneExpendKalmanFilterSubscriber::realtime_distance_left_callback, this, std::placeholders::_1)
        );

        // 创建订阅者，订阅 "/start_torque_correction" 话题
        start_torque_correction_sub = this->create_subscription<std_msgs::msg::Bool>(
            "/start_torque_correction", 10,  // 话题名称和队列大小
            std::bind(&MultipleCallbackOneExpendKalmanFilterSubscriber::start_torque_correction_callback, this, std::placeholders::_1)
        );

        timer_ = this->create_wall_timer(
            std::chrono::nanoseconds(500),
            std::bind(&MultipleCallbackOneExpendKalmanFilterSubscriber::timer_callback, this)
        );

        theta_from_expend_kalman_filter_pub = this->create_publisher<geometry_msgs::msg::Twist>("/theta_from_expend_kalman_filter", 10);
        theta_from_joint_position_with_noise_pub = this->create_publisher<geometry_msgs::msg::Twist>("/theta_from_joint_position_with_noise", 10);
        theta_from_joint_position_pub = this->create_publisher<geometry_msgs::msg::Twist>("/theta_from_joint_position", 10);
        theta_velocity_from_joint_velocity_pub = this->create_publisher<geometry_msgs::msg::Twist>("/theta_velocity_expend_kalman_filter", 10);
        effort_from_joint_efforts_with_noise_pub = this->create_publisher<geometry_msgs::msg::Twist>("/effort_from_joint_efforts_with_noise", 10);
        expend_kalman_filter_result_k_1_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/expend_kalman_filter_result_k_1", 10);
        z_axis_error_pub = this->create_publisher<std_msgs::msg::Float64>("/z_axis_error", 10);
        send_update_pub = this->create_publisher<std_msgs::msg::Bool>("/send_update", 10);
    }

private:
    // 回调函数：处理 "/joint_state" 话题的消息
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        
        if (start_next_expend_kalman_filter == false && start_expend_kalman_filter)
        {
            return;
        }

        // 确保消息中包含足够的关节数据
        if (msg->position.size() < 6 || msg->velocity.size() < 6 || msg->effort.size() < 6) {
            RCLCPP_ERROR(this->get_logger(), "Joint state message does not contain enough data.");
            return;
        }

        // 计算时间间隔dt
        dt_outside = 0.008; // 默认值
        if (last_time.sec != 0 || last_time.nanosec != 0) {
            // 计算时间差（秒）
            double current_time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
            double previous_time = last_time.sec + last_time.nanosec * 1e-9;
            dt_outside = current_time - previous_time;
            
            // 限制dt的范围，避免异常值
            if (dt_outside <= 0 || dt_outside > 1) {
                dt_outside = 0.008; // 使用默认值
                RCLCPP_WARN(this->get_logger(), "Invalid dt calculated: %f, using default value: 0.008", dt_outside);
            }
        }

        // 提取关节位置数据
        Eigen::MatrixXd joint_positions = Eigen::MatrixXd::Zero(6, 1);
        joint_positions << msg->position[5], msg->position[0], msg->position[1], msg->position[2], msg->position[3], msg->position[4];      
        Eigen::MatrixXd joint_velocitys = Eigen::MatrixXd::Zero(6, 1);
        joint_velocitys << msg->velocity[5], msg->velocity[0], msg->velocity[1], msg->velocity[2], msg->velocity[3], msg->velocity[4];
        Eigen::MatrixXd joint_efforts = Eigen::MatrixXd::Zero(6, 1);
        joint_efforts << msg->effort[5], msg->effort[0], msg->effort[1], msg->effort[2], msg->effort[3], msg->effort[4];

        // 关节电流转换为关节力矩
        joint_efforts(0, 0) = joint_efforts(0, 0) * 13.13;
        joint_efforts(1, 0) = joint_efforts(1, 0) * 13.13;
        joint_efforts(2, 0) = joint_efforts(2, 0) * 9.3122;
        joint_efforts(3, 0) = joint_efforts(3, 0) * 4.545;
        joint_efforts(4, 0) = joint_efforts(4, 0) * 4.545;
        joint_efforts(5, 0) = joint_efforts(5, 0) * 4.545;
    
        geometry_msgs::msg::Twist theta_from_joint_position;

        theta_from_joint_position.linear.x = joint_positions(0, 0);
        theta_from_joint_position.linear.y = joint_positions(1, 0);
        theta_from_joint_position.linear.z = joint_positions(2, 0);
        theta_from_joint_position.angular.x = joint_positions(3, 0);
        theta_from_joint_position.angular.y = joint_positions(4, 0);
        theta_from_joint_position.angular.z = joint_positions(5, 0);

        theta_from_joint_position_pub->publish(theta_from_joint_position);

        // geometry_msgs::msg::Twist theta_from_joint_position_with_noise;
        // theta_from_joint_position_with_noise.linear.x = joint_positions(0, 0);
        // theta_from_joint_position_with_noise.linear.y = joint_positions(1, 0);
        // theta_from_joint_position_with_noise.linear.z = joint_positions(2, 0);
        // theta_from_joint_position_with_noise.angular.x = joint_positions(3, 0);
        // theta_from_joint_position_with_noise.angular.y = joint_positions(4, 0);
        // theta_from_joint_position_with_noise.angular.z = joint_positions(5, 0);
        // theta_from_joint_position_with_noise_pub->publish(theta_from_joint_position_with_noise);

        // geometry_msgs::msg::Twist effort_from_joint_efforts_with_noise;
        // effort_from_joint_efforts_with_noise.linear.x = joint_efforts(0, 0);
        // effort_from_joint_efforts_with_noise.linear.y = joint_efforts(1, 0);
        // effort_from_joint_efforts_with_noise.linear.z = joint_efforts(2, 0);
        // effort_from_joint_efforts_with_noise.angular.x = joint_efforts(3, 0);
        // effort_from_joint_efforts_with_noise.angular.y = joint_efforts(4, 0);
        // effort_from_joint_efforts_with_noise.angular.z = joint_efforts(5, 0);
        // effort_from_joint_efforts_with_noise_pub->publish(effort_from_joint_efforts_with_noise);

        if (start_torque_correction) {
            number_of_correction++;
            joint_positions_correction = joint_positions_correction + (joint_positions - joint_positions_correction) / number_of_correction;
            joint_velocitys_correction = joint_velocitys_correction + (joint_velocitys - joint_velocitys_correction) / number_of_correction;
            joint_efforts_correction = joint_efforts_correction + (joint_efforts - joint_efforts_correction) / number_of_correction;
            
            if (number_of_correction >= 1000) {
                start_torque_correction = false;
                whether_torque_correction = true;
                number_of_correction = 0;

                Eigen::MatrixXd tau = Eigen::MatrixXd::Zero(6, 1);
                Eigen::MatrixXd F_screw_tip = CalculateF_Screw_Tip(joint_transform_matrix(joint_positions_correction));
                tau = NewtonEulerInverseDynamics(joint_positions_correction, Eigen::MatrixXd::Zero(6, 1), Eigen::MatrixXd::Zero(6, 1), F_screw_tip, true, joint_transform_matrix(joint_positions_correction));
                effort_correction = tau - joint_efforts_correction;

                Z_Axis_zero_position = Z_Axis_calculate(joint_positions_correction) + laser_data * 0.001 * 0.001;

                RCLCPP_INFO(this->get_logger(), "Torque correction finished");
                RCLCPP_INFO(this->get_logger(), "Z_Axis_zero_position = %f", Z_Axis_zero_position);
                // RCLCPP_INFO(this->get_logger(), "joint_positions_correction = %f, %f, %f, %f, %f, %f", joint_positions_correction(0, 0), joint_positions_correction(1, 0), joint_positions_correction(2, 0), joint_positions_correction(3, 0), joint_positions_correction(4, 0), joint_positions_correction(5, 0));
                // RCLCPP_INFO(this->get_logger(), "joint_velocitys_correction = %f, %f, %f, %f, %f, %f", joint_velocitys_correction(0, 0), joint_velocitys_correction(1, 0), joint_velocitys_correction(2, 0), joint_velocitys_correction(3, 0), joint_velocitys_correction(4, 0), joint_velocitys_correction(5, 0));
                // RCLCPP_INFO(this->get_logger(), "joint_efforts_correction = %f, %f, %f, %f, %f, %f", joint_efforts_correction(0, 0), joint_efforts_correction(1, 0), joint_efforts_correction(2, 0), joint_efforts_correction(3, 0), joint_efforts_correction(4, 0), joint_efforts_correction(5, 0));
                // RCLCPP_INFO(this->get_logger(), "effort_correction = %f, %f, %f, %f, %f, %f", effort_correction(0, 0), effort_correction(1, 0), effort_correction(2, 0), effort_correction(3, 0), effort_correction(4, 0), effort_correction(5, 0));
            }

        }

        if (start_expend_kalman_filter && whether_torque_correction) {
            start_next_expend_kalman_filter = false;
            theta_k_outside = joint_positions;
            theta_velocity_k_outside = joint_velocitys;
            
            expend_kalman_filter_result_k_1.X_posterior = expend_kalman_filter_result_k.X_posterior;
            expend_kalman_filter_result_k_1.P_posterior = expend_kalman_filter_result_k.P_posterior;

            send_update_callback = false;
            std_msgs::msg::Bool msg_send_update;
            msg_send_update.data = true;
            send_update_pub->publish(msg_send_update);
            std::this_thread::sleep_for(std::chrono::nanoseconds(50));

            // while(send_update_callback == false) {
            //     RCLCPP_INFO(this->get_logger(), "send_update_callback = false");
            //     std::this_thread::sleep_for(std::chrono::nanoseconds(50));
            // }

            std_msgs::msg::Float64MultiArray expend_kalman_filter_result_k_1_msg;
            expend_kalman_filter_result_k_1_msg.data.resize(19);

            for (int i = 0; i < 6; i++) {
                expend_kalman_filter_result_k_1_msg.data[i] = expend_kalman_filter_result_k_1.X_posterior(i, 0);
                expend_kalman_filter_result_k_1_msg.data[i + 6] = expend_kalman_filter_result_k_1.X_posterior(i + 6, 0);
                expend_kalman_filter_result_k_1_msg.data[i + 12] = torque_k_1(i, 0);
            }
            expend_kalman_filter_result_k_1_msg.data[18] = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
            expend_kalman_filter_result_k_1_pub->publish(expend_kalman_filter_result_k_1_msg);
            
            calculateExpendKalmanFilter_prior(torque_k_1, dt_outside);
            torque_k_1 = joint_efforts + effort_correction;

        } else {
            expend_kalman_filter_result_k.X_posterior.block<6, 1>(0, 0) = joint_positions;
            expend_kalman_filter_result_k.X_posterior.block<6, 1>(6, 0) = joint_velocitys;
            torque_k_1 = joint_efforts + effort_correction;
            expend_kalman_filter_result_k.P_posterior = Eigen::MatrixXd::Identity(12, 12) * 0.0001;
            expend_kalman_filter_result_k.P_posterior.block<6, 6>(0, 0) << 0.00002234 * 0.00002234, 0, 0, 0, 0, 0,
                                                                       0, 0.00002192 * 0.00002192, 0, 0, 0, 0,
                                                                       0, 0, 0.00002274 * 0.00002274, 0, 0, 0,
                                                                       0, 0, 0, 0.0000184 * 0.0000184, 0, 0,
                                                                       0, 0, 0, 0, 0.000019 * 0.000019, 0,
                                                                       0, 0, 0, 0, 0, 0.00002184 * 0.00002184;
        }
        
        // 更新上一次的时间戳
        last_time = msg->header.stamp;
    }

    void start_expend_kalman_filter_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        start_expend_kalman_filter = msg->data;
    }

    void start_debug_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
            RCLCPP_INFO(this->get_logger(), "Start debug");
            Eigen::MatrixXd T_i_1_i_assemble = Eigen::MatrixXd::Zero(4, 4 * 7);
            T_i_1_i_assemble = joint_transform_matrix(expend_kalman_filter_result_k.X_posterior.block<6, 1>(0, 0));
            Eigen::MatrixXd tau = Eigen::MatrixXd::Zero(6, 1);
            tau = NewtonEulerInverseDynamics(expend_kalman_filter_result_k.X_posterior.block<6, 1>(0, 0), expend_kalman_filter_result_k.X_posterior.block<6, 1>(6, 0), Eigen::MatrixXd::Zero(6, 1), Eigen::MatrixXd::Zero(6, 1), true, T_i_1_i_assemble);
            RCLCPP_INFO(this->get_logger(), "tau = %f, %f, %f, %f, %f, %f", tau(0, 0), tau(1, 0), tau(2, 0), tau(3, 0), tau(4, 0), tau(5, 0));
        } else {
            RCLCPP_INFO(this->get_logger(), "Stop debug");
        }
    }

    void callback2_expend_kalman_filter_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        
        A_callback = Eigen::MatrixXd::Zero(12, 12);
        A_callback.block<6, 6>(0, 0) = Eigen::MatrixXd::Zero(6, 6);
        A_callback.block<6, 6>(0, 6) = Eigen::MatrixXd::Identity(6, 6);

        for (int i = 1; i <= 6; i++) {
            A_callback.block<6, 1>(6, i-1) << msg->data[6 * (i-1)], msg->data[6 * (i-1) + 1], msg->data[6 * (i-1) + 2], msg->data[6 * (i-1) + 3], msg->data[6 * (i-1) + 4], msg->data[6 * (i-1) + 5];
        }
        for (int i = 1; i <= 6; i++) {
            A_callback.block<6, 1>(6, i+5) << msg->data[36 + 6 * (i-1)], msg->data[36 + 6 * (i-1) + 1], msg->data[36 + 6 * (i-1) + 2], msg->data[36 + 6 * (i-1) + 3], msg->data[36 + 6 * (i-1) + 4], msg->data[36 + 6 * (i-1) + 5];
        }
        Ad_callback = Eigen::MatrixXd::Identity(12, 12) + A_callback * dt_outside;

        Z_Axis_callback = Eigen::MatrixXd::Zero(1, 7);
        Z_Axis_callback << msg->data[72], msg->data[73], msg->data[74], msg->data[75], msg->data[76], msg->data[77], msg->data[78];
        
        callback2_expend_kalman_filter_result = true;
    }

    void send_update_callback_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        send_update_callback = true;
    }   

    void timer_callback() {
        if (callback2_expend_kalman_filter_result) {
            callback2_expend_kalman_filter_result = false;
            calculateExpendKalmanFilter_posterior(Ad_callback, dt_outside);

            geometry_msgs::msg::Twist theta_from_expend_kalman_filter;
            theta_from_expend_kalman_filter.linear.x = expend_kalman_filter_result_k.X_posterior(0, 0);
            theta_from_expend_kalman_filter.linear.y = expend_kalman_filter_result_k.X_posterior(1, 0);
            theta_from_expend_kalman_filter.linear.z = expend_kalman_filter_result_k.X_posterior(2, 0);
            theta_from_expend_kalman_filter.angular.x = expend_kalman_filter_result_k.X_posterior(3, 0);
            theta_from_expend_kalman_filter.angular.y = expend_kalman_filter_result_k.X_posterior(4, 0);
            theta_from_expend_kalman_filter.angular.z = expend_kalman_filter_result_k.X_posterior(5, 0);

            geometry_msgs::msg::Twist theta_velocity_from_joint_velocity;
            theta_velocity_from_joint_velocity.linear.x = expend_kalman_filter_result_k.X_posterior(6, 0);
            theta_velocity_from_joint_velocity.linear.y = expend_kalman_filter_result_k.X_posterior(7, 0);
            theta_velocity_from_joint_velocity.linear.z = expend_kalman_filter_result_k.X_posterior(8, 0);
            theta_velocity_from_joint_velocity.angular.x = expend_kalman_filter_result_k.X_posterior(9, 0);
            theta_velocity_from_joint_velocity.angular.y = expend_kalman_filter_result_k.X_posterior(10, 0);
            theta_velocity_from_joint_velocity.angular.z = expend_kalman_filter_result_k.X_posterior(11, 0);
            
            theta_velocity_from_joint_velocity_pub->publish(theta_velocity_from_joint_velocity);

            // if (first_time) {
            //     first_time = false;
            //     std::cout << "expend_kalman_filter_result_k.X_posterior:" << std::endl;
            //     std::cout << expend_kalman_filter_result_k.X_posterior << std::endl;
            //     std::cout << "expend_kalman_filter_result_k.P_posterior:" << std::endl;
            //     std::cout << expend_kalman_filter_result_k.P_posterior << std::endl;
            // }

            // // 检查卡尔曼滤波输出结果是否异常
            // bool has_abnormal_output = false;
            // if (std::abs(theta_from_expend_kalman_filter.linear.x) > 10.0 ||
            //     std::abs(theta_from_expend_kalman_filter.linear.y) > 10.0 ||
            //     std::abs(theta_from_expend_kalman_filter.linear.z) > 10.0 ||
            //     std::abs(theta_from_expend_kalman_filter.angular.x) > 10.0 ||
            //     std::abs(theta_from_expend_kalman_filter.angular.y) > 10.0 ||
            //     std::abs(theta_from_expend_kalman_filter.angular.z) > 10.0) {
            //     has_abnormal_output = true;
            // }
            
            // if (has_abnormal_output) {
            //     RCLCPP_ERROR(this->get_logger(), "Abnormal Kalman filter output detected:");
            //     RCLCPP_ERROR(this->get_logger(), "theta_k = [%f, %f, %f, %f, %f, %f]", 
            //                 theta_k_outside(0, 0), theta_k_outside(1, 0), theta_k_outside(2, 0), theta_k_outside(3, 0), theta_k_outside(4, 0), theta_k_outside(5, 0));
            // }

            theta_from_expend_kalman_filter_pub->publish(theta_from_expend_kalman_filter);
            start_next_expend_kalman_filter = true;
        }
    }

    void realtime_distance_left_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        laser_data = msg->data;    
        realtime_z_from_laser = Z_Axis_zero_position - msg->data * 0.001 * 0.001;
    }

    void start_torque_correction_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        start_torque_correction = msg->data;
        if (start_torque_correction) {
            RCLCPP_INFO(this->get_logger(), "Start torque correction");
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_expend_kalman_filter_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_debug_sub;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr callback2_expend_kalman_filter_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr send_update_callback_sub;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr realtime_distance_left_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_torque_correction_sub;
    builtin_interfaces::msg::Time last_time; // 添加成员变量来存储上一次的时间戳
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);  // 初始化 ROS 2
    auto node = std::make_shared<MultipleCallbackOneExpendKalmanFilterSubscriber>();  // 创建订阅者节点
    
    // 创建多线程执行器
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    
    // 使用多线程执行器运行节点
    executor.spin();  // 保持节点运行，监听消息
    
    rclcpp::shutdown();  // 关闭 ROS 2
    return 0;
}