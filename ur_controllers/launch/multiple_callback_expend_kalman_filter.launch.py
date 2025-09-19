from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    nodes = []
    
    nodes.append(
        Node(
            package='ur_controllers',  # 替换为你的包名
            executable='multiple_callback_one_expend_kalman_filter_node',  # 替换为你的可执行文件名
            name='kalman_filter_one',
            output="screen"
        )   
    )

    nodes.append(
        Node(
            package='ur_controllers',  # 替换为你的包名
            executable='multiple_callback_two_expend_kalman_filter_node',  # 替换为你的可执行文件名
            name='kalman_filter_two',
            output="screen"
        )
    )

    nodes.append(
        Node(
            package='ur_controllers',  # 替换为你的包名
            executable='multiple_callback_three_expend_kalman_filter_node',  # 替换为你的可执行文件名
            name='kalman_filter_three',
            output="screen"
        )
    )

    nodes.append(
        Node(
            package='ur_controllers',  # 替换为你的包名
            executable='multiple_callback_four_expend_kalman_filter_node',  # 替换为你的可执行文件名
            name='kalman_filter_four',
            output="screen"
        )
    )

    nodes.append(
        Node(
            package='ur_controllers',  # 替换为你的包名
            executable='multiple_callback_five_expend_kalman_filter_node',  # 替换为你的可执行文件名
            name='kalman_filter_five',
            output="screen"
        )
    )

    nodes.append(
        Node(
            package='ur_controllers',  # 替换为你的包名
            executable='multiple_callback_six_expend_kalman_filter_node',  # 替换为你的可执行文件名
            name='kalman_filter_six',   
            output="screen"
        )
    )

    nodes.append(
        Node(
            package='ur_controllers',  # 替换为你的包名
            executable='multiple_callback_seven_expend_kalman_filter_node',  # 替换为你的可执行文件名
            name='kalman_filter_seven',
            output="screen"
        )
    )   

    nodes.append(
        Node(
            package='ur_controllers',  # 替换为你的包名
            executable='multiple_callback_eight_expend_kalman_filter_node',  # 替换为你的可执行文件名
            name='kalman_filter_eight',
            output="screen"
        )
    )

    nodes.append(
        Node(
            package='ur_controllers',  # 替换为你的包名
            executable='multiple_callback_nine_expend_kalman_filter_node',  # 替换为你的可执行文件名
            name='kalman_filter_nine',
            output="screen"
        )
    )

    nodes.append(
        Node(
            package='ur_controllers',  # 替换为你的包名
            executable='multiple_callback_ten_expend_kalman_filter_node',  # 替换为你的可执行文件名
            name='kalman_filter_ten',
            output="screen"
        )   
    )   

    nodes.append(
        Node(
            package='ur_controllers',  # 替换为你的包名
            executable='multiple_callback_eleven_expend_kalman_filter_node',  # 替换为你的可执行文件名
            name='kalman_filter_eleven',
            output="screen"
        )   
    )   

    return LaunchDescription(nodes)