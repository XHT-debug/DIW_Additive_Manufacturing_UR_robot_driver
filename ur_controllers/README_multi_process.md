# 多进程扩展卡尔曼滤波器

## 概述

这是一个使用10个进程进行计算的扩展卡尔曼滤波器实现，用于UR机器人的状态估计。相比单进程版本，多进程版本能够更好地利用多核CPU资源，提高计算效率。

## 主要特性

- **10个工作线程**: 使用10个独立的工作线程进行卡尔曼滤波计算
- **线程安全**: 使用互斥锁和条件变量确保线程安全
- **异步处理**: 数据接收和计算分离，提高响应速度
- **负载均衡**: 多个工作线程可以并行处理不同的计算任务

## 架构设计

### 主要组件

1. **主线程**: 负责ROS2消息的接收和发布
2. **工作线程池**: 10个工作线程，每个线程独立执行卡尔曼滤波计算
3. **数据队列**: 线程安全的数据队列，用于存储待处理的数据
4. **结果管理**: 线程安全的结果存储和获取机制

### 工作流程

1. 主线程接收关节状态数据
2. 将数据添加到线程安全队列
3. 通知工作线程处理数据
4. 工作线程执行卡尔曼滤波计算
5. 将结果存储到线程安全数据结构
6. 主线程获取计算结果并发布

## 编译和运行

### 编译

```bash
cd ur_driver_workspace
colcon build --packages-select ur_controllers
```

### 运行

#### 方法1: 使用启动文件
```bash
ros2 launch ur_controllers expend_kalman_filter_multi_process.launch.py
```

#### 方法2: 直接运行节点
```bash
ros2 run ur_controllers expend_kalman_filter_multi_process_node
```

## 话题

### 订阅话题
- `/joint_states` (sensor_msgs/msg/JointState): 关节状态数据
- `/start_expend_kalman_filter` (std_msgs/msg/Bool): 启动/停止卡尔曼滤波器
- `/start_debug` (std_msgs/msg/Bool): 启动/停止调试模式

### 发布话题
- `/theta_from_expend_kalman_filter` (geometry_msgs/msg/Twist): 卡尔曼滤波器输出
- `/theta_from_joint_position_with_noise` (geometry_msgs/msg/Twist): 带噪声的关节位置
- `/theta_from_joint_position` (geometry_msgs/msg/Twist): 原始关节位置

## 性能优化

### 线程数量配置

可以通过修改源代码中的 `NUM_WORKERS` 常量来调整工作线程数量：

```cpp
const int NUM_WORKERS = 10;  // 修改为所需的工作线程数量
```

### 内存管理

- 使用智能指针管理内存
- 避免不必要的内存拷贝
- 使用引用传递减少开销

### 计算优化

- 矩阵运算使用Eigen库的优化实现
- 避免重复计算
- 使用条件变量减少CPU占用

## 故障排除

### 常见问题

1. **编译错误**: 确保已安装所有依赖包
2. **运行时错误**: 检查ROS2环境是否正确设置
3. **性能问题**: 根据CPU核心数调整工作线程数量

### 调试

启用调试模式：
```bash
ros2 topic pub /start_debug std_msgs/msg/Bool "data: true"
```

## 与单进程版本的区别

| 特性 | 单进程版本 | 多进程版本 |
|------|------------|------------|
| 线程数量 | 1 | 10 |
| 计算方式 | 同步 | 异步 |
| 响应速度 | 较慢 | 较快 |
| 资源占用 | 较低 | 较高 |
| 复杂度 | 简单 | 复杂 |

## 注意事项

1. 多进程版本需要更多的系统资源
2. 确保系统有足够的CPU核心支持多线程
3. 在高负载情况下可能需要调整线程数量
4. 建议在测试环境中先验证性能表现 