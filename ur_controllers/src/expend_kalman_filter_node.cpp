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
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <builtin_interfaces/msg/time.hpp>


rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr theta_from_joint_position_pub;
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr theta_from_joint_position_with_noise_pub;
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr theta_from_expend_kalman_filter_pub;
expend_kalman_filter_result expend_kalman_filter_result_k;
expend_kalman_filter_result expend_kalman_filter_result_k_1;
bool start_expend_kalman_filter = false;
bool first_time = true;
Eigen::MatrixXd torque_k_1 = Eigen::MatrixXd::Zero(6, 1);

void calculateExpendKalmanFilter(const Eigen::MatrixXd& torque_k_1, const double& dt, const Eigen::MatrixXd& theta_k)
{
    Eigen::MatrixXd X_k_prior = Eigen::MatrixXd::Zero(12, 1); //先验状态估计
    Eigen::MatrixXd X_k_posterior = Eigen::MatrixXd::Zero(12, 1); //后验状态估计
    Eigen::MatrixXd P_k_prior = Eigen::MatrixXd::Zero(12, 12); // 先验估计协方差矩阵
    Eigen::MatrixXd P_k_posterior = Eigen::MatrixXd::Zero(12, 12); // 后验估计协方差矩阵
    Eigen::MatrixXd K_k = Eigen::MatrixXd::Zero(12, 6); // 卡尔曼增益矩阵
    Eigen::MatrixXd H_k = Eigen::MatrixXd::Zero(6, 12); // 观测矩阵
    Eigen::MatrixXd R_k = Eigen::MatrixXd::Zero(6, 6); // 观测噪声协方差矩阵
    Eigen::MatrixXd Q_k = Eigen::MatrixXd::Zero(12, 12); // 过程噪声协方差矩阵


    R_k.block<6, 6>(0, 0) << 0.00002234 * 0.00002234, 0, 0, 0, 0, 0,
                             0, 0.00002192 * 0.00002192, 0, 0, 0, 0,
                             0, 0, 0.00002274 * 0.00002274, 0, 0, 0,
                             0, 0, 0, 0.0000184 * 0.0000184, 0, 0,
                             0, 0, 0, 0, 0.000019 * 0.000019, 0,
                             0, 0, 0, 0, 0, 0.00002184 * 0.00002184;
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
    Q_k = 2.5 * Q_k;
    R_k = 2500 * R_k;

    H_k.block<6, 6>(0, 0) = Eigen::MatrixXd::Identity(6, 6);
    H_k.block<6, 6>(0, 6) = Eigen::MatrixXd::Zero(6, 6);


    // 计算先验状态估计
    X_k_prior.block<6, 1>(0, 0) = expend_kalman_filter_result_k_1.X_posterior.block<6, 1>(0, 0) + expend_kalman_filter_result_k_1.X_posterior.block<6, 1>(6, 0) * dt;
    X_k_prior.block<6, 1>(6, 0) = expend_kalman_filter_result_k_1.X_posterior.block<6, 1>(6, 0) + NewtonEulerForwardDynamics(expend_kalman_filter_result_k_1.X_posterior.block<6, 1>(0, 0), expend_kalman_filter_result_k_1.X_posterior.block<6, 1>(6, 0), torque_k_1) * dt;


    // 计算离散时间状态转移矩阵Ad
    Eigen::MatrixXd Ad = DiscreteStateSpaceModel(expend_kalman_filter_result_k_1.X_posterior.block<6, 1>(0, 0), expend_kalman_filter_result_k_1.X_posterior.block<6, 1>(6, 0), torque_k_1, dt);
    
    if (first_time)
    {
        RCLCPP_INFO(rclcpp::get_logger("expend_kalman_filter"), "Ad:");
        std::cout << Ad << std::endl;
        first_time = false;
    }

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

    Eigen::MatrixXd H_k_theta_k = H_k * X_k_prior;
    // 计算后验状态估计
    X_k_posterior = X_k_prior + K_k * (theta_k - H_k * X_k_prior);
    
    // 计算后验估计协方差矩阵P_k_posterior
    P_k_posterior = (Eigen::MatrixXd::Identity(12, 12) - K_k * H_k) * P_k_prior;

    expend_kalman_filter_result_k.X_posterior = X_k_posterior;
    expend_kalman_filter_result_k.P_posterior = P_k_posterior;

    // // 验证结果是否包含NaN或无穷大
    // if (!expend_kalman_filter_result_k.X_posterior.allFinite()) {
    //     RCLCPP_ERROR(rclcpp::get_logger("expend_kalman_filter"), "X_posterior contains NaN or infinite values, returning previous result");
    //     return expend_kalman_filter_result_k_1;
    // }
    
    // if (!expend_kalman_filter_result_k.P_posterior.allFinite()) {
    //     RCLCPP_ERROR(rclcpp::get_logger("expend_kalman_filter"), "P_posterior contains NaN or infinite values, returning previous result");
    //     return expend_kalman_filter_result_k_1;
    // }
}

double gaussian_perturbation() {
    // 使用 static 保证引擎和分布只初始化一次，避免重复播种
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::normal_distribution<double> dist(0.0, 0.00002);  // 均值0，标准差0.00002

    return dist(gen);
}

class ExpendKalmanFilterSubscriber : public rclcpp::Node
{
public:
    ExpendKalmanFilterSubscriber()
        : Node("expend_kalman_filter")  // 设置节点名称
    {
        // 创建订阅者，订阅 "/joint_states" 话题
        joint_states_sub = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,  // 话题名称和队列大小
            std::bind(&ExpendKalmanFilterSubscriber::joint_state_callback, this, std::placeholders::_1));

        // 创建订阅者，订阅 "/start_expend_kalman_filter" 话题
        start_expend_kalman_filter_sub = this->create_subscription<std_msgs::msg::Bool>(
            "/start_expend_kalman_filter", 10,  // 话题名称和队列大小
            std::bind(&ExpendKalmanFilterSubscriber::start_expend_kalman_filter_callback, this, std::placeholders::_1));

        // 创建订阅者，订阅 "/start_debug" 话题 
        start_debug_sub = this->create_subscription<std_msgs::msg::Bool>(
            "/start_debug", 10,  // 话题名称和队列大小
            std::bind(&ExpendKalmanFilterSubscriber::start_debug_callback, this, std::placeholders::_1));

        theta_from_expend_kalman_filter_pub = this->create_publisher<geometry_msgs::msg::Twist>("/theta_from_expend_kalman_filter", 10);
        theta_from_joint_position_with_noise_pub = this->create_publisher<geometry_msgs::msg::Twist>("/theta_from_joint_position_with_noise", 10);
        theta_from_joint_position_pub = this->create_publisher<geometry_msgs::msg::Twist>("/theta_from_joint_position", 10);

    }

private:
    // 回调函数：处理 "/joint_state" 话题的消息
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        
        // 确保消息中包含足够的关节数据
        if (msg->position.size() < 6 || msg->velocity.size() < 6 || msg->effort.size() < 6) {
            RCLCPP_ERROR(this->get_logger(), "Joint state message does not contain enough data.");
            return;
        }

        // 计算时间间隔dt
        double dt = 0.008; // 默认值
        if (last_time.sec != 0 || last_time.nanosec != 0) {
            // 计算时间差（秒）
            double current_time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
            double previous_time = last_time.sec + last_time.nanosec * 1e-9;
            dt = current_time - previous_time;
            
            // 限制dt的范围，避免异常值
            if (dt <= 0 || dt > 1) {
                dt = 0.008; // 使用默认值
                RCLCPP_WARN(this->get_logger(), "Invalid dt calculated: %f, using default value: 0.008", dt);
            }
        }

        // 提取关节位置数据
        Eigen::MatrixXd joint_positions = Eigen::MatrixXd::Zero(6, 1);
        joint_positions << msg->position[5], msg->position[0], msg->position[1], msg->position[2], msg->position[3], msg->position[4];      
        Eigen::MatrixXd joint_velocitys = Eigen::MatrixXd::Zero(6, 1);
        joint_velocitys << msg->velocity[5], msg->velocity[0], msg->velocity[1], msg->velocity[2], msg->velocity[3], msg->velocity[4];
        Eigen::MatrixXd joint_efforts = Eigen::MatrixXd::Zero(6, 1);
        joint_efforts << msg->effort[5], msg->effort[0], msg->effort[1], msg->effort[2], msg->effort[3], msg->effort[4];

        joint_efforts(0, 0) = joint_efforts(0, 0) * 13.13;
        joint_efforts(1, 0) = joint_efforts(1, 0) * 13.13;
        joint_efforts(2, 0) = joint_efforts(2, 0) * 9.3122;
        joint_efforts(3, 0) = joint_efforts(3, 0) * 4.545;
        joint_efforts(4, 0) = joint_efforts(4, 0) * 4.545;
        joint_efforts(5, 0) = joint_efforts(5, 0) * 4.545;
    
        // joint_efforts(0, 0) = (joint_efforts(0, 0) - (-0.247386)) * 0 + (-0.247386);
        // joint_efforts(1, 0) = (joint_efforts(1, 0) - (1.065195)) * 0 + (1.065195);
        // joint_efforts(2, 0) = (joint_efforts(2, 0) - (0.766035)) * 0 + (0.766035);
        // joint_efforts(3, 0) = (joint_efforts(3, 0) - (0.299693)) * 0 + (0.299693);
        // joint_efforts(4, 0) = (joint_efforts(4, 0) - (-0.015606)) * 0 + (-0.015606);
        // joint_efforts(5, 0) = (joint_efforts(5, 0) - (0.006004)) * 0 + (0.006004);
        geometry_msgs::msg::Twist theta_from_joint_position;
        theta_from_joint_position.linear.x = joint_positions(0, 0);
        theta_from_joint_position.linear.y = joint_positions(1, 0);
        theta_from_joint_position.linear.z = joint_positions(2, 0);
        theta_from_joint_position.angular.x = joint_positions(3, 0);
        theta_from_joint_position.angular.y = joint_positions(4, 0);
        theta_from_joint_position.angular.z = joint_positions(5, 0);
        theta_from_joint_position_pub->publish(theta_from_joint_position);

        // joint_positions(0, 0) += gaussian_perturbation();
        // joint_positions(1, 0) += gaussian_perturbation();
        // joint_positions(2, 0) += gaussian_perturbation();
        // joint_positions(3, 0) += gaussian_perturbation();
        // joint_positions(4, 0) += gaussian_perturbation();
        // joint_positions(5, 0) += gaussian_perturbation();

        geometry_msgs::msg::Twist theta_from_joint_position_with_noise;
        theta_from_joint_position_with_noise.linear.x = joint_positions(0, 0);
        theta_from_joint_position_with_noise.linear.y = joint_positions(1, 0);
        theta_from_joint_position_with_noise.linear.z = joint_positions(2, 0);
        theta_from_joint_position_with_noise.angular.x = joint_positions(3, 0);
        theta_from_joint_position_with_noise.angular.y = joint_positions(4, 0);
        theta_from_joint_position_with_noise.angular.z = joint_positions(5, 0);
        theta_from_joint_position_with_noise_pub->publish(theta_from_joint_position_with_noise);

        if (start_expend_kalman_filter) {
            Eigen::MatrixXd theta_k = Eigen::MatrixXd::Zero(6, 1);
            theta_k = joint_positions;
            
            expend_kalman_filter_result_k_1.X_posterior = expend_kalman_filter_result_k.X_posterior;
            expend_kalman_filter_result_k_1.P_posterior = expend_kalman_filter_result_k.P_posterior;

            calculateExpendKalmanFilter(torque_k_1, dt, theta_k);

            torque_k_1 = joint_efforts;

            geometry_msgs::msg::Twist theta_from_expend_kalman_filter;
            theta_from_expend_kalman_filter.linear.x = expend_kalman_filter_result_k.X_posterior(0, 0);
            theta_from_expend_kalman_filter.linear.y = expend_kalman_filter_result_k.X_posterior(1, 0);
            theta_from_expend_kalman_filter.linear.z = expend_kalman_filter_result_k.X_posterior(2, 0);
            theta_from_expend_kalman_filter.angular.x = expend_kalman_filter_result_k.X_posterior(3, 0);
            theta_from_expend_kalman_filter.angular.y = expend_kalman_filter_result_k.X_posterior(4, 0);
            theta_from_expend_kalman_filter.angular.z = expend_kalman_filter_result_k.X_posterior(5, 0);

            // 检查卡尔曼滤波输出结果是否异常
            bool has_abnormal_output = false;
            if (std::abs(theta_from_expend_kalman_filter.linear.x) > 10.0 ||
                std::abs(theta_from_expend_kalman_filter.linear.y) > 10.0 ||
                std::abs(theta_from_expend_kalman_filter.linear.z) > 10.0 ||
                std::abs(theta_from_expend_kalman_filter.angular.x) > 10.0 ||
                std::abs(theta_from_expend_kalman_filter.angular.y) > 10.0 ||
                std::abs(theta_from_expend_kalman_filter.angular.z) > 10.0) {
                has_abnormal_output = true;
            }
            
            if (has_abnormal_output) {
                RCLCPP_ERROR(this->get_logger(), "Abnormal Kalman filter output detected:");
                RCLCPP_ERROR(this->get_logger(), "theta_k = [%f, %f, %f, %f, %f, %f]", 
                            theta_k(0, 0), theta_k(1, 0), theta_k(2, 0), theta_k(3, 0), theta_k(4, 0), theta_k(5, 0));
            }

            theta_from_expend_kalman_filter_pub->publish(theta_from_expend_kalman_filter);

        } else {
            expend_kalman_filter_result_k.X_posterior.block<6, 1>(0, 0) = joint_positions;
            expend_kalman_filter_result_k.X_posterior.block<6, 1>(6, 0) = joint_velocitys;
            torque_k_1 = joint_efforts;
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

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_expend_kalman_filter_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_debug_sub;
    builtin_interfaces::msg::Time last_time; // 添加成员变量来存储上一次的时间戳
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);  // 初始化 ROS 2
    auto node = std::make_shared<ExpendKalmanFilterSubscriber>();  // 创建订阅者节点
    
    // 创建多线程执行器
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    
    // 使用多线程执行器运行节点
    executor.spin();  // 保持节点运行，监听消息
    
    rclcpp::shutdown();  // 关闭 ROS 2
    return 0;
}