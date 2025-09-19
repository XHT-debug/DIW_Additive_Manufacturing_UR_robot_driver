import numpy as np
import math

# 全局变量定义
PI = 3.14159265358979323846
alpha_dynamic = None
a_dynamic = None
d_dynamic = None

def dynamic_setup():
    """初始化DH参数(UR3机械臂参数)"""
    global alpha_dynamic, a_dynamic, d_dynamic
    
    alpha_dynamic = np.array([PI/2, 0, 0, PI/2, -PI/2, 0])
    a_dynamic = np.array([0, -0.24376288177085403, -0.21344066092027106, 0, 0, 0])
    d_dynamic = np.array([0.15187159190950295, 0, 0, 0.11209388124617316, 0.085378607490435937, 0.08241227224463675])

def joint_transform_matrix(theta):
    """
    计算关节变换矩阵
    
    参数:
    theta: numpy数组,形状为(6, 1)或(6,),包含6个关节角度
    
    返回:
    T_i_1_i_assemble: numpy数组,形状为(4, 28),包含所有关节的变换矩阵
    """
    dynamic_setup()
    
    # 确保theta是numpy数组且形状正确
    if isinstance(theta, list):
        theta = np.array(theta)
    if theta.ndim == 1:
        theta = theta.reshape(-1, 1)
    
    T_i_1_i_assemble = np.zeros((4, 4 * 7))
    
    for i in range(1, 7):  # i从1到6
        cos_theta = math.cos(math.fmod(theta[i-1, 0], 2 * PI))
        sin_theta = math.sin(math.fmod(theta[i-1, 0], 2 * PI))
        cos_alpha = math.cos(math.fmod(alpha_dynamic[i-1], 2 * PI))
        sin_alpha = math.sin(math.fmod(alpha_dynamic[i-1], 2 * PI))
        
        # 构建变换矩阵
        T_matrix = np.array([
            [cos_theta, -sin_theta * cos_alpha, sin_theta * sin_alpha, a_dynamic[i-1] * cos_theta],
            [sin_theta, cos_theta * cos_alpha, -cos_theta * sin_alpha, a_dynamic[i-1] * sin_theta],
            [0, sin_alpha, cos_alpha, d_dynamic[i-1]],
            [0, 0, 0, 1]
        ])
        
        # 将变换矩阵放入组装矩阵中
        T_i_1_i_assemble[0:4, 4*(i-1):4*i] = T_matrix
    
    # 添加第7个变换矩阵（单位矩阵）
    T_i_1_i_assemble[0:4, 4*6:4*7] = np.eye(4)
    
    return T_i_1_i_assemble

def zuobiaozhuanhuan_calculate(theta):
    """
    计算TCP的Z轴坐标
    
    参数:
    theta: numpy数组,形状为(6, 1)或(6,),包含6个关节角度
    
    返回:
    float: TCP的Z轴坐标
    """
    dynamic_setup()
    T_i_1_i_asb = joint_transform_matrix(theta)
    
    # 初始化TCP位置向量
    tcp = np.eye(4)
    
    # 从第6个关节开始向前计算
    for i in range(6, 0, -1):  # i从6到1
        T_matrix = T_i_1_i_asb[0:4, 4*(i-1):4*i]
        tcp = T_matrix @ tcp
    
    return tcp

def rotation_matrix_to_quaternion(R):
    """
    将一个3x3的旋转矩阵R转换为单位四元数q = [w, x, y, z]
    """
    # 计算分量的平方
    t = np.trace(R)
    w_squared = (1 + t) / 4
    x_squared = (1 + 2*R[0, 0] - t) / 4
    y_squared = (1 + 2*R[1, 1] - t) / 4
    z_squared = (1 + 2*R[2, 2] - t) / 4

    # 找到最大的平方值
    squares = np.array([w_squared, x_squared, y_squared, z_squared])
    index = np.argmax(squares)

    # 根据最大值计算分量
    if index == 0: # w最大
        w = math.sqrt(w_squared)
        s = 0.25 / w
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif index == 1: # x最大
        x = math.sqrt(x_squared)
        s = 0.25 / x
        w = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 1] + R[1, 0]) * s
        z = (R[0, 2] + R[2, 0]) * s
    elif index == 2: # y最大
        y = math.sqrt(y_squared)
        s = 0.25 / y
        w = (R[0, 2] - R[2, 0]) * s
        x = (R[0, 1] + R[1, 0]) * s
        z = (R[1, 2] + R[2, 1]) * s
    else: # z最大
        z = math.sqrt(z_squared)
        s = 0.25 / z
        w = (R[1, 0] - R[0, 1]) * s
        x = (R[0, 2] + R[2, 0]) * s
        y = (R[1, 2] + R[2, 1]) * s

    q = np.array([w, x, y, z])
    # 规范化以确保是单位四元数（通常已经很接近，但为了安全）
    q = q / np.linalg.norm(q)
    return q

# 测试代码
if __name__ == "__main__":
    # 测试用例
    # test_theta = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6]).reshape(-1, 1)
    test_theta = np.array([0.19079560041427612, -1.775545899068014, -1.5410588423358362, -1.395287815724508, 1.569222331047058, 0.19516348838806152])

    zuobiaozhuanhuan = zuobiaozhuanhuan_calculate(test_theta)

    test_r = np.array(zuobiaozhuanhuan[0:3, 0:3])
    test_q = rotation_matrix_to_quaternion(test_r)

    print("测试关节角度:", test_theta.flatten())
    print("TCP坐标:", zuobiaozhuanhuan_calculate(test_theta))
    print("旋转矩阵:", test_r)
    print("四元数:", test_q)
    