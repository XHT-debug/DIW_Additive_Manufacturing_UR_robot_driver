#include "ur_controllers/dynamics_statespace.hpp"
#include <iostream>

// 全局变量定义
double g = 9.81; // 重力加速度
const double PI = 3.14159265358979323846; // 双精度
Eigen::VectorXd alpha_dynamic(6);
Eigen::VectorXd a_dynamic(6);
Eigen::VectorXd d_dynamic(6);
Eigen::MatrixXd m_assemble(1, 6);
Eigen::MatrixXd centroid_vector_assemble(3, 6);
Eigen::MatrixXd A_Screw_assemble(6, 6);
double m_tip = 0.290; // 末端质量，待测量
Eigen::MatrixXd centroid_vector_tip(3, 1); // 末端质心，待测量，以坐标系{6}为参考
double theta_epsilon = 1e-4; // 关节角度微分步长
double dtheta_epsilon = 1e-4; // 关节角速度微分步长
bool whether_print_M_theta = true; // 是否输出M_theta矩阵的所有元素值
bool whether_print = true; // 是否输出M_theta矩阵的所有元素值

// 拓展卡尔曼滤波初始化
// 目标：初始化DH参数，质量参数，质心向量，旋量轴矩阵
// 输入：无
// 输出：无
void dynamic_setup()
{
    //  初始化DH参数（UR3机械臂参数）
    alpha_dynamic << PI/2, 0, 0, PI/2, -PI/2, 0;
    a_dynamic << 0, -0.24376288177085403, -0.21344066092027106, 0, 0, 0;
    d_dynamic << 0.15187159190950295, 0, 0, 0.11209388124617316, 0.085378607490435937, 0.08241227224463675;
    
    // 初始化质量参数
    m_assemble << 2.000, 3.420, 1.260, 0.800, 0.800, 0.350;
    
    // 初始化质心向量
    centroid_vector_assemble <<     0,    0.13,   0.05,    0,    0,     0,
                                 -0.02,       0,      0,    0,    0,     0,
                                     0, 0.01157, 0.0238, 0.01, 0.01, -0.02;
    
    // 初始化旋量轴矩阵
    A_Screw_assemble << 0,       0,       0,  0,  0, 0,
                        1,       0,       0, -1,  1, 0,  
                        0,      -1,      -1,  0,  0, 1,
                        0,       0,       0,  0,  0, 0,
                        0, 0.24365, 0.21325,  0,  0, 0,
                        0,       0,       0,  0,  0, 0;

    centroid_vector_tip << 0, -0.028, 0.049;
    
}

// 反对称矩阵计算函数
// 目标：计算向量的反对称矩阵
// 输入：向量
// 输出：反对称矩阵
Eigen::MatrixXd skewSymmetric(const Eigen::MatrixXd& v) {
    Eigen::MatrixXd m = Eigen::MatrixXd::Zero(3, 3);
    m << 0,    -v(2, 0), v(1, 0),
         v(2, 0), 0,    -v(0, 0),
         -v(1, 0), v(0, 0), 0;
    return m;
}

// 伴随变换矩阵计算函数
// 目标：计算坐标系转换矩阵的伴随变换矩阵，与常规的伴随矩阵不同
// 输入：坐标系转换矩阵
// 输出：伴随变换矩阵
Eigen::MatrixXd Ad_T(const Eigen::MatrixXd& T)
{
    Eigen::MatrixXd Ad_T = Eigen::MatrixXd::Zero(6, 6);
    Eigen::MatrixXd R = T.block<3, 3>(0, 0);
    Eigen::MatrixXd p = T.block<3, 1>(0, 3);
    Ad_T.block<3, 3>(0, 0) = R;
    Ad_T.block<3, 3>(3, 3) = R;
    Ad_T.block<3, 3>(3, 0) = skewSymmetric(p) * R;
    return Ad_T;
}

// 关节转换矩阵计算函数
// 目标：当前关节角度下的关节转换矩阵
// 输入：关节角度
// 输出：关节转换矩阵
Eigen::MatrixXd joint_transform_matrix(const Eigen::MatrixXd& theta)
{
    dynamic_setup();
    Eigen::MatrixXd T_i_1_i_assemble = Eigen::MatrixXd::Zero(4, 4 * 7);
    for (int i = 1; i <= 6; i++) {
        double cos_theta = std::cos(std::fmod(theta(i-1, 0), 2 * PI));
        double sin_theta = std::sin(std::fmod(theta(i-1, 0), 2 * PI));
        double cos_alpha = std::cos(std::fmod(alpha_dynamic(i-1, 0), 2 * PI));
        double sin_alpha = std::sin(std::fmod(alpha_dynamic(i-1, 0), 2 * PI));
        T_i_1_i_assemble.block<4, 4>(0, 4 * (i - 1)) << cos_theta, -sin_theta * cos_alpha, sin_theta * sin_alpha, a_dynamic(i-1) * cos_theta,
                                                        sin_theta, cos_theta * cos_alpha, -cos_theta * sin_alpha, a_dynamic(i-1) * sin_theta,
                                                        0, sin_alpha, cos_alpha, d_dynamic(i-1),
                                                        0, 0, 0, 1;
    }
    T_i_1_i_assemble.block<4, 4>(0, 4 * 6) << 1, 0, 0, 0,
                                            0, 1, 0, 0,
                                            0, 0, 1, 0,
                                            0, 0, 0, 1;


    return T_i_1_i_assemble;
}

// 末端力旋量计算函数
// 目标：当前关节角度下的末端力旋量
// 输入：关节转换矩阵
// 输出：末端力旋量
Eigen::MatrixXd CalculateF_Screw_Tip(const Eigen::MatrixXd& T_i_1_i_asb)
{
    dynamic_setup();
    Eigen::MatrixXd F_Screw_tip = Eigen::MatrixXd::Zero(6, 1);

    // 计算不同姿态下末端喷嘴等附加设备在末端坐标系下的力旋量
    Eigen::MatrixXd T_0_6 = T_i_1_i_asb.block<4, 4>(0, 0) * T_i_1_i_asb.block<4, 4>(0, 4 * 1) * T_i_1_i_asb.block<4, 4>(0, 4 * 2) * T_i_1_i_asb.block<4, 4>(0, 4 * 3) * T_i_1_i_asb.block<4, 4>(0, 4 * 4) * T_i_1_i_asb.block<4, 4>(0, 4 * 5);
    Eigen::MatrixXd g_ = Eigen::MatrixXd::Zero(3, 1);
    g_ << 0, 0, -g;
    Eigen::MatrixXd g_tip = T_0_6.block<3, 3>(0, 0) * g_;
    F_Screw_tip.block<3, 1>(0, 0) = m_tip * g_tip;
    F_Screw_tip.block<3, 1>(3, 0) = skewSymmetric(centroid_vector_tip) * F_Screw_tip.block<3, 1>(0, 0);

    return F_Screw_tip;
}


// 雅可比矩阵计算函数
// 目标：当前关节角度下的雅可比矩阵
// 输入：关节角度、关节转换矩阵、关节转换矩阵的伴随矩阵
// 输出：雅可比矩阵
Eigen::MatrixXd CalculateJacobian(const Eigen::MatrixXd& theta, const Eigen::MatrixXd& T_i_1_i_assemble)
{
    dynamic_setup();
    Eigen::MatrixXd Jacobian = Eigen::MatrixXd::Zero(6, 6);
    Eigen::MatrixXd T_0_i = Eigen::MatrixXd::Zero(4, 4);
    Eigen::MatrixXd T_0_i_1 = Eigen::MatrixXd::Identity(4, 4);

    for (int i = 1; i <= 6; i++) {
        T_0_i = T_0_i_1 * T_i_1_i_assemble.block<4, 4>(0, 4 * (i - 1));
        T_0_i_1 = T_0_i;
        Jacobian.block<6, 1>(0, i - 1) = Ad_T(T_0_i) * A_Screw_assemble.block<6, 1>(0, i - 1);
    }

    return Jacobian;
}

// 雅可比矩阵逆计算函数
// 目标：当前关节角度下的雅可比矩阵逆
// 输入：雅可比矩阵
// 输出：雅可比矩阵逆
Eigen::MatrixXd CalculateJacobianInverse(const Eigen::MatrixXd& Jacobian)
{
    return Jacobian.transpose() * ((Jacobian * Jacobian.transpose()).inverse());
}

// 牛顿-欧拉逆动力学函数
// 目标：实现ur六轴机械臂的逆向动力学计算
// 输入：关节角度、关节速度、关节加速度
// 输出：关节力矩
Eigen::MatrixXd NewtonEulerInverseDynamics(const Eigen::MatrixXd& theta, const Eigen::MatrixXd& dtheta, const Eigen::MatrixXd& d2theta, const Eigen::MatrixXd& F_Screw_tip, const bool whether_gravity, const Eigen::MatrixXd& T_i_1_i_assemble)
{
    dynamic_setup();
    Eigen::MatrixXd V_Screw_assemble = Eigen::MatrixXd::Zero(6, 1 * 7); 
    Eigen::MatrixXd dV_Screw_assemble = Eigen::MatrixXd::Zero(6, 1 * 7); 
    Eigen::MatrixXd F_Screw_assemble = Eigen::MatrixXd::Zero(6, 1 * 7);
    Eigen::MatrixXd tau = Eigen::MatrixXd::Zero(6, 1);

    // 初始化矩阵集合
    V_Screw_assemble.block<6, 1>(0, 0) = Eigen::MatrixXd::Zero(6, 1);
    if (whether_gravity) 
    {
        dV_Screw_assemble.block<6, 1>(0, 0) << 0, 0, 0, 0, 0, -g;
    }else
    {
        dV_Screw_assemble.block<6, 1>(0, 0) << 0, 0, 0, 0, 0, 0;
    }
    F_Screw_assemble.block<6, 1>(0, 6) = F_Screw_tip;

    // 正向迭代
    for (int i = 1; i <= 6; i++) {
        Eigen::MatrixXd T_i_i_1 = T_i_1_i_assemble.block<4, 4>(0, 4 * (i - 1)).inverse();
        Eigen::MatrixXd A_Screw_i = A_Screw_assemble.block<6, 1>(0, i - 1);
        Eigen::MatrixXd ad_V_Screw_i_forward = Eigen::MatrixXd::Zero(6, 6);

        Eigen::MatrixXd V_Screw_i = Ad_T(T_i_i_1) * V_Screw_assemble.block<6, 1>(0, i - 1) + A_Screw_i * dtheta(i - 1, 0);
        V_Screw_assemble.block<6, 1>(0, i) = V_Screw_i;

        ad_V_Screw_i_forward.block<3, 3>(0, 0) = skewSymmetric(V_Screw_i.block<3, 1>(0, 0));
        ad_V_Screw_i_forward.block<3, 3>(3, 0) = skewSymmetric(V_Screw_i.block<3, 1>(3, 0));
        ad_V_Screw_i_forward.block<3, 3>(3, 3) = ad_V_Screw_i_forward.block<3, 3>(0, 0);

        Eigen::MatrixXd dV_Screw_i = Ad_T(T_i_i_1) * dV_Screw_assemble.block<6, 1>(0, i - 1) + ad_V_Screw_i_forward * A_Screw_i * dtheta(i - 1, 0) + A_Screw_i * d2theta(i - 1, 0);
        dV_Screw_assemble.block<6, 1>(0, i) = dV_Screw_i;
    }
    
    // 逆向迭代
    for (int i = 6; i >= 1; i--) {
        Eigen::MatrixXd spatial_inertia_i = Eigen::MatrixXd::Zero(6, 6);
        Eigen::MatrixXd T_i__1_i = T_i_1_i_assemble.block<4, 4>(0, 4 * i).inverse();
        
        Eigen::MatrixXd I_cm = Eigen::MatrixXd::Zero(3, 3);
        Eigen::MatrixXd centroid_vector_i = centroid_vector_assemble.block<3, 1>(0, i - 1);

        Eigen::MatrixXd ad_V_Screw_i_inverse = Eigen::MatrixXd::Zero(6, 6);
        ad_V_Screw_i_inverse.block<3, 3>(0, 0) = skewSymmetric(V_Screw_assemble.block<3, 1>(0, i));
        ad_V_Screw_i_inverse.block<3, 3>(3, 0) = skewSymmetric(V_Screw_assemble.block<3, 1>(3, i));
        ad_V_Screw_i_inverse.block<3, 3>(3, 3) = ad_V_Screw_i_inverse.block<3, 3>(0, 0);
        
        // 优化
        spatial_inertia_i.block<3, 3>(0, 0) = I_cm + m_assemble(0, i-1) * ((centroid_vector_i.transpose() * centroid_vector_i)(0, 0) * Eigen::MatrixXd::Identity(3, 3) - centroid_vector_i * centroid_vector_i.transpose());
        spatial_inertia_i.block<3, 3>(3, 0) = m_assemble(0, i-1) * skewSymmetric(centroid_vector_i).transpose();
        spatial_inertia_i.block<3, 3>(0, 3) = m_assemble(0, i-1) * skewSymmetric(centroid_vector_i);
        spatial_inertia_i.block<3, 3>(3, 3) <<  m_assemble(0, i-1), 0, 0,
                                                0, m_assemble(0, i-1), 0,
                                                0, 0, m_assemble(0, i-1);

        // spatial_inertia_i = Ad_T(T_i__1_i) * spatial_inertia_i * T_i__1_i;
        Eigen::MatrixXd F_Screw_i__1 = F_Screw_assemble.block<6, 1>(0, i);

        Eigen::MatrixXd F_Screw_i = Ad_T(T_i__1_i).transpose() * F_Screw_i__1 + spatial_inertia_i * dV_Screw_assemble.block<6, 1>(0, i) - ad_V_Screw_i_inverse.transpose() * spatial_inertia_i * V_Screw_assemble.block<6, 1>(0, i);
        F_Screw_assemble.block<6, 1>(0, i - 1) = F_Screw_i;

        tau(i - 1, 0) = (F_Screw_i.transpose() * A_Screw_assemble.block<6, 1>(0, i - 1)).value();
    }

    return tau;
}

// 牛顿-欧拉正动力学函数
// 目标：实现ur六轴机械臂的正向动力学计算
// 输入：关节角度、关节速度、关节力矩
// 输出：关节加速度
Eigen::MatrixXd NewtonEulerForwardDynamics(const Eigen::MatrixXd& theta, const Eigen::MatrixXd& dtheta, const Eigen::MatrixXd& tau)
{
    dynamic_setup();
    Eigen::MatrixXd d2theta(6, 1);
    Eigen::MatrixXd M_theta(6, 6);
    Eigen::MatrixXd h_theta_dtheta(6, 1);
    Eigen::MatrixXd J_theta(6, 6);
    Eigen::MatrixXd zero = Eigen::MatrixXd::Zero(6, 1);
    Eigen::MatrixXd T_i_1_i_asb = joint_transform_matrix(theta);
    Eigen::MatrixXd F_Screw_tip = CalculateF_Screw_Tip(T_i_1_i_asb);



    // 计算惯性矩阵M(theta)
    for (int i = 1; i <= 6; i++) {
        Eigen::MatrixXd d2theta_i = Eigen::MatrixXd::Zero(6, 1);
        d2theta_i(i - 1, 0) = 1;
        M_theta.block<6, 1>(0, i - 1) = NewtonEulerInverseDynamics(theta, dtheta, d2theta_i, zero, false, T_i_1_i_asb);
    }

    // 计算科里奥利力矩,离心力矩,重力力矩 h(theta, dtheta)
    h_theta_dtheta = NewtonEulerInverseDynamics(theta, dtheta, zero, zero, true, T_i_1_i_asb);

    // 计算雅可比矩阵 J(theta)
    J_theta = CalculateJacobian(theta, T_i_1_i_asb);

    // 求解线性方程组 M(theta) * d2theta = tau - h(theta, dtheta) - J_theta.transpose() * F_Screw_tip
    Eigen::MatrixXd b = tau - h_theta_dtheta - J_theta.transpose() * F_Screw_tip;
    // if (whether_print)
    // {
    //     std::cout << "b:" << std::endl;
    //     std::cout << b << std::endl;
    // }
    d2theta = truncated_svd_solve(M_theta, b);

    return d2theta;
}

// 六轴机械臂的连续时间状态空间模型计算
// 目标：实现ur六轴机械臂的连续时间状态空间模型计算
// 输入：关节角度、关节速度、关节力矩
// 输出：状态空间模型
Eigen::MatrixXd ContinuousStateSpaceModel(const Eigen::MatrixXd& theta, const Eigen::MatrixXd& dtheta, const Eigen::MatrixXd& tau)
{
    dynamic_setup();
    // 状态空间模型为：
    // x_dot = A * x + B * u
    // y = C * x + D * u
    // 其中x为状态变量，u为输入变量，y为输出变量
    // A为状态空间矩阵，B为输入矩阵，C为输出矩阵，D为直接传递矩阵
    // 定义状态变量x为关节角度和关节速度，即x = [theta, dtheta]
    // 定义输入变量u为关节力矩，即u = tau
    // 定义输出变量y为关节加速度，即y = d2theta
    // 输出：状态空间模型

    // 系统在某角度下的状态空间模型
    Eigen::MatrixXd A(12, 12);
    // Eigen::MatrixXd B(12, 6);
    // Eigen::MatrixXd C(6, 12);
    // Eigen::MatrixXd D(6, 6);

    Eigen::MatrixXd T_i_1_i_asb = joint_transform_matrix(theta);
    Eigen::MatrixXd M_theta(6, 6);
    A.block<6, 6>(0, 0) = Eigen::MatrixXd::Zero(6, 6);
    A.block<6, 6>(0, 6) = Eigen::MatrixXd::Identity(6, 6);

    // 计算惯性矩阵M(theta)
    for (int i = 1; i <= 6; i++) {
        Eigen::MatrixXd d2theta_i = Eigen::MatrixXd::Zero(6, 1);
        d2theta_i(i - 1, 0) = 1;
        M_theta.block<6, 1>(0, i - 1) = NewtonEulerInverseDynamics(theta, dtheta, d2theta_i, Eigen::MatrixXd::Zero(6, 1), false, T_i_1_i_asb);
    }


    // 计算当前关节角度，角速度的系统矩阵，通过牛顿欧拉动力学计算
    // 系统动力学方程为：M(theta) * d2theta = tau - h(theta, dtheta) - J_theta.transpose() * F_Screw_tip
    // 1.计算每个关节角度对关节角加速度的微分，微分布长设置1e-6,采用2阶中心差分法
    for (int i = 1; i <= 6; i++) {
        Eigen::MatrixXd theta_perturb = theta;
        theta_perturb(i - 1) += theta_epsilon;
        Eigen::MatrixXd d2theta_perturb1 = NewtonEulerForwardDynamics(theta_perturb, dtheta, tau);
        theta_perturb(i - 1) -= 2 * theta_epsilon;
        Eigen::MatrixXd d2theta_perturb2 = NewtonEulerForwardDynamics(theta_perturb, dtheta, tau);
        A.block<6, 1>(6, i - 1) = (d2theta_perturb1 - d2theta_perturb2) / (2 * theta_epsilon);
    }

    // 2.计算每个关节角度对关节角速度的微分，微分布长设置1e-6,采用2阶中心差分法
    // 动力学方程中仅有h(theta, dtheta)对dtheta的微分，因此需要计算h(theta, dtheta)对dtheta的微分,减少计算量
    for (int i = 1; i <= 6; i++) {
        Eigen::MatrixXd dtheta_perturb = dtheta;
        dtheta_perturb(i - 1) += dtheta_epsilon;
        // 计算科里奥利力矩,离心力矩,重力力矩 h(theta, dtheta)
        Eigen::MatrixXd h_theta_dtheta_perturb1 = NewtonEulerInverseDynamics(theta, dtheta_perturb, Eigen::MatrixXd::Zero(6, 1), Eigen::MatrixXd::Zero(6, 1), true, T_i_1_i_asb);
        dtheta_perturb(i - 1) -= 2 * dtheta_epsilon;
        Eigen::MatrixXd h_theta_dtheta_perturb2 = NewtonEulerInverseDynamics(theta, dtheta_perturb, Eigen::MatrixXd::Zero(6, 1), Eigen::MatrixXd::Zero(6, 1), true, T_i_1_i_asb);
        A.block<6, 1>(6, 6 + i - 1) = truncated_svd_solve(M_theta, h_theta_dtheta_perturb1 - h_theta_dtheta_perturb2) / (2 * dtheta_epsilon);
    }

    return A;
}

// 六轴机械臂的离散时间状态空间模型计算
// 目标：实现ur六轴机械臂的离散时间状态空间模型计算
// 输入：关节角度、关节速度、关节力矩
// 输出：离散时间状态空间模型
Eigen::MatrixXd DiscreteStateSpaceModel(const Eigen::MatrixXd& theta, const Eigen::MatrixXd& dtheta, const Eigen::MatrixXd& tau, const double& dt)
{
    dynamic_setup();
    // 计算连续时间状态空间模型系统矩阵A
    Eigen::MatrixXd A = ContinuousStateSpaceModel(theta, dtheta, tau);
    // if (whether_print)
    // {
    //     std::cout << "A:" << std::endl;
    //     std::cout << A << std::endl;
    //     whether_print = false;
    // }

    // 采用一阶保持(First-Order Hold)方法将连续时间状态空间模型离散化
    // 计算离散时间状态空间模型系统矩阵Ad
    Eigen::MatrixXd Ad = Eigen::MatrixXd::Identity(12, 12) + A * dt;

    return Ad;    
}

Eigen::MatrixXd truncated_svd_solve(const Eigen::MatrixXd& input_A, const Eigen::MatrixXd& input_b)
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(input_A, 
                                     Eigen::ComputeThinU | 
                                     Eigen::ComputeThinV);
    Eigen::MatrixXd U = svd.matrixU();
    Eigen::MatrixXd V = svd.matrixV();
    Eigen::VectorXd S = svd.singularValues();
    double tol;

    // 确认需要截断的奇异值阈值
    double max_s = S(0);
    // if (tol < 0)
    // {
    //     tol = std::max(input_A.rows(), input_A.cols()) * std::numeric_limits<double>::epsilon() * max_s;
    // }
    tol = std::max(input_A.rows(), input_A.cols()) * std::numeric_limits<double>::epsilon() * max_s;
    // // 确认需要保留的奇异值
    // int r = 0;
    // for (int i = 0; i < S.size(); i++)
    // {
    //     if (S(i) > tol)
    //     {
    //         r++;
    //     }
    // }

    // // 诊断信息
    // std::cout << "SVD诊断:\n";
    // std::cout << "  矩阵大小: " << input_A.rows() << " x " << input_A.cols() << "\n";
    // std::cout << "  最大奇异值: " << std::scientific << S(0) << "\n";
    // std::cout << "  最小奇异值: " << std::scientific << S(S.size()-1) << "\n";
    // std::cout << "  截断阈值: " << std::scientific << tol << "\n";
    // std::cout << "  保留奇异值数: " << r << " (共 " << S.size() << ")\n";
    
    // 计算截断解
    Eigen::MatrixXd x_truncated = Eigen::MatrixXd::Zero(input_A.cols(), 1);
    for (int i = 0; i < S.size(); ++i)
    {
        if (S(i) > tol)
        {
            x_truncated += (V.col(i).transpose() * input_b)(0, 0) * U.col(i) / S(i);
        }
    }

    return x_truncated;
}


double Z_Axis_calculate(const Eigen::MatrixXd& theta)
{
    dynamic_setup();
    Eigen::MatrixXd T_i_1_i_asb = joint_transform_matrix(theta);
    Eigen::MatrixXd tcp = Eigen::MatrixXd::Zero(4, 1);
    tcp << 0, 0, 0, 1;

    for (int i = 6; i >= 1; i--) {
        tcp = T_i_1_i_asb.block<4, 4>(0, 4 * (i - 1)) * tcp;
    }

    return tcp(2, 0);
}

Eigen::MatrixXd Z_Axis_observation_matrix_calculate(const Eigen::MatrixXd& theta)
{
    Eigen::MatrixXd Z_Axis_observation_matrix_i = Eigen::MatrixXd::Zero(6, 1);
    double epsilon = 1e-5;

    for (int i = 1; i <= 6; i++) {
        Eigen::MatrixXd theta_perturb1 = Eigen::MatrixXd::Zero(6, 1);
        theta_perturb1 = theta;
        theta_perturb1(i-1, 0) += 2 * epsilon;
        double f_plus2 = Z_Axis_calculate(theta_perturb1);

        Eigen::MatrixXd theta_perturb2 = Eigen::MatrixXd::Zero(6, 1);
        theta_perturb2 = theta;
        theta_perturb2(i-1, 0) += epsilon;
        double f_plus1 = Z_Axis_calculate(theta_perturb2);

        Eigen::MatrixXd theta_perturb3 = Eigen::MatrixXd::Zero(6, 1);
        theta_perturb3 = theta;
        theta_perturb3(i-1, 0) -= epsilon;
        double f_minus1 = Z_Axis_calculate(theta_perturb3);

        Eigen::MatrixXd theta_perturb4 = Eigen::MatrixXd::Zero(6, 1);
        theta_perturb4 = theta;
        theta_perturb4(i-1, 0) -= 2 * epsilon;
        double f_minus2 = Z_Axis_calculate(theta_perturb4);

        // 五点中心差分公式
        Z_Axis_observation_matrix_i(i-1, 0) = (-f_plus2 + 8*f_plus1 - 8*f_minus1 + f_minus2) / (12 * epsilon);
    }
    
    return Z_Axis_observation_matrix_i;
}