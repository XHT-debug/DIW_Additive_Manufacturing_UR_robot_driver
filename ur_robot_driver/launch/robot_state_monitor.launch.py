#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 声明启动参数
    tf_prefix_arg = DeclareLaunchArgument(
        'tf_prefix',
        default_value='',
        description='TF prefix for the robot'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='125.0',
        description='Publish rate in Hz'
    )
    
    # 获取参数值
    tf_prefix = LaunchConfiguration('tf_prefix')
    publish_rate = LaunchConfiguration('publish_rate')
    
    # 创建机器人状态监控节点
    robot_state_monitor_node = Node(
        package='ur_robot_driver',
        executable='robot_state_monitor',
        name='robot_state_monitor',
        output='screen',
        parameters=[{
            'tf_prefix': tf_prefix,
            'publish_rate': publish_rate,
        }]
    )
    
    return LaunchDescription([
        tf_prefix_arg,
        publish_rate_arg,
        robot_state_monitor_node,
    ])