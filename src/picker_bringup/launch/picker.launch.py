"""전체 실행.

시뮬레이션 (하드웨어 없이):
  ros2 launch picker_bringup picker.launch.py mode:=sim
실제 로봇 (RealSense, doosan-robot2는 각각 먼저 실행):
  ros2 launch picker_bringup picker.launch.py mode:=real gripper_mode:=drl
비교 실험:
  ros2 launch picker_bringup picker.launch.py mode:=sim strategy:=random
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('picker_bringup'), 'config')
    mode = LaunchConfiguration('mode')
    is_sim = PythonExpression(["'", mode, "' == 'sim'"])
    poses = os.path.join(cfg, 'poses.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='sim', description='sim | real'),
        DeclareLaunchArgument('strategy', default_value='priority', description='priority | color_order | random'),
        DeclareLaunchArgument('gripper_mode', default_value='sim', description='sim | drl'),
        DeclareLaunchArgument('orders_file', default_value=os.path.join(cfg, 'orders.yaml')),
        DeclareLaunchArgument('seed', default_value='0', description='sim 모드 큐브 배치 난수'),

        # ---- 인식: sim이면 가짜 작업대, real이면 vision_node ----
        Node(package='picker_vision', executable='fake_world', condition=IfCondition(is_sim),
             parameters=[{'seed': LaunchConfiguration('seed')}], output='screen'),
        Node(package='picker_vision', executable='vision_node', condition=UnlessCondition(is_sim),
             parameters=[{'vision_config': os.path.join(cfg, 'vision.yaml'),
                          'calibration_file': os.path.join(cfg, 'calibration.yaml')}],
             output='screen'),

        # ---- 제어 ----
        Node(package='picker_control', executable='robot_controller', output='screen',
             parameters=[{'poses_file': poses,
                          'robot_mode': PythonExpression(["'sim' if '", mode, "' == 'sim' else 'dsr'"]),
                          'gripper_mode': LaunchConfiguration('gripper_mode')}]),

        # ---- 계획 ----
        Node(package='picker_planner', executable='task_planner', output='screen',
             parameters=[{'poses_file': poses, 'strategy': LaunchConfiguration('strategy')}]),
        Node(package='picker_planner', executable='order_manager', output='screen',
             parameters=[{'orders_file': LaunchConfiguration('orders_file')}]),
    ])
