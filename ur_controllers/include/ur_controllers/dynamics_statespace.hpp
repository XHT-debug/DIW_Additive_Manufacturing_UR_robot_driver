#ifndef UR_CONTROLLERS__DYNAMICS_STATESPACE_HPP_
#define UR_CONTROLLERS__DYNAMICS_STATESPACE_HPP_

#include <vector>
#include <cmath>
#include <chrono>
#include <Eigen/Dense>
#include <Eigen/SVD>

struct expend_kalman_filter_result
{
    Eigen::MatrixXd X_posterior = Eigen::MatrixXd::Zero(12, 1);
    Eigen::MatrixXd P_posterior = Eigen::MatrixXd::Zero(12, 12);
};

// 六轴机械臂DH参数 
extern double g; // 重力加速度
extern Eigen::VectorXd alpha_dynamic;
extern Eigen::VectorXd a_dynamic;
extern Eigen::VectorXd d_dynamic;
extern Eigen::MatrixXd m_assemble;
extern Eigen::MatrixXd centroid_vector_assemble;
extern Eigen::MatrixXd A_Screw_assemble;
extern double m_tip; // 末端质量，待测量
extern Eigen::MatrixXd centroid_vector_tip; // 末端质心，待测量，以坐标系{6}为参考
extern double theta_epsilon; // 关节角度微分步长
extern double dtheta_epsilon; // 关节角速度微分步长
extern double dt; // 离散时间步长

// 拓展卡尔曼滤波初始化
// 目标：初始化DH参数，质量参数，质心向量，旋量轴矩阵
// 输入：无
// 输出：无
void dynamic_setup();  

// 反对称矩阵计算函数
// 目标：计算向量的反对称矩阵
// 输入：向量
// 输出：反对称矩阵
Eigen::MatrixXd skewSymmetric(const Eigen::MatrixXd& v);

// 雅可比矩阵计算函数
// 目标：当前关节角度下的雅可比矩阵
// 输入：关节角度、关节转换矩阵、关节转换矩阵的伴随矩阵
// 输出：雅可比矩阵
Eigen::MatrixXd CalculateJacobian(const Eigen::MatrixXd& theta, const Eigen::MatrixXd& T_i_1_i_assemble);

// 雅可比矩阵逆计算函数
// 目标：当前关节角度下的雅可比矩阵逆
// 输入：雅可比矩阵
// 输出：雅可比矩阵逆
Eigen::MatrixXd CalculateJacobianInverse(const Eigen::MatrixXd& Jacobian);

// 伴随变换矩阵计算函数
// 目标：计算坐标系转换矩阵的伴随变换矩阵，与常规的伴随矩阵不同
// 输入：坐标系转换矩阵
// 输出：伴随变换矩阵
Eigen::MatrixXd Ad_T(const Eigen::MatrixXd& T);

// 关节坐标系转换矩阵计算函数
// 目标：计算关节坐标系转换矩阵
// 输入：关节角度
// 输出：关节坐标系转换矩阵
Eigen::MatrixXd joint_transform_matrix(const Eigen::MatrixXd& theta);

// 逆动力学计算函数
// 目标：计算逆动力学，根据关节角度、角速度、角加速度和末端力计算关节力矩
// 输入：关节角度、关节角速度、关节角加速度、末端力、是否考虑重力、关节变换矩阵
// 输出：关节力矩
Eigen::MatrixXd NewtonEulerInverseDynamics(const Eigen::MatrixXd& theta, const Eigen::MatrixXd& dtheta, const Eigen::MatrixXd& d2theta, const Eigen::MatrixXd& F_Screw_tip, const bool whether_gravity, const Eigen::MatrixXd& T_i_1_i_assemble);

// 末端力矩计算函数
// 目标：计算末端力矩
// 输入：关节角度，关节角速度，关节力矩
// 输出：末端力矩
Eigen::MatrixXd NewtonEulerForwardDynamics(const Eigen::MatrixXd& theta, const Eigen::MatrixXd& dtheta, const Eigen::MatrixXd& tau);

// 六轴机械臂的连续时间状态空间模型计算
// 目标：实现ur六轴机械臂的连续时间状态空间模型计算
// 输入：关节角度、关节速度、关节力矩
// 输出：状态空间模型
Eigen::MatrixXd ContinuousStateSpaceModel(const Eigen::MatrixXd& theta, const Eigen::MatrixXd& dtheta, const Eigen::MatrixXd& tau);

// 六轴机械臂的离散时间状态空间模型计算
// 目标：实现ur六轴机械臂的离散时间状态空间模型计算
// 输入：关节角度、关节速度、关节力矩
// 输出：状态空间模型
Eigen::MatrixXd DiscreteStateSpaceModel(const Eigen::MatrixXd& theta, const Eigen::MatrixXd& dtheta, const Eigen::MatrixXd& tau, const double& dt);

// 截断奇异值分解求解函数
// 目标：实现截断奇异值分解求解
// 输入：矩阵，向量，截断阈值
// 输出：解
Eigen::MatrixXd truncated_svd_solve(const Eigen::MatrixXd& input_A, const Eigen::MatrixXd& input_b);

// 末端位置Z轴计算函数
// 目标：计算末端位置Z轴
// 输入：关节角度
// 输出：末端位置Z轴
double Z_Axis_calculate(const Eigen::MatrixXd& theta);

// 末端位置Z轴观测矩阵计算函数
// 目标：计算末端位置Z轴观测矩阵
// 输入：关节角度
// 输出：末端位置Z轴观测矩阵
Eigen::MatrixXd Z_Axis_observation_matrix_calculate(const Eigen::MatrixXd& theta);

// 末端力旋量计算函数
// 目标：计算末端力旋量
// 输入：关节转换矩阵
// 输出：末端力旋量
Eigen::MatrixXd CalculateF_Screw_Tip(const Eigen::MatrixXd& T_i_1_i_asb);

// 雅可比矩阵计算函数

#endif // UR_CONTROLLERS__DYNAMICS_STATESPACE_HPP_ 