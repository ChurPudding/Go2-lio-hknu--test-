import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('go2_description')
    urdf_path = os.path.join(pkg_share, 'urdf', 'go2_description.urdf')
    rviz_path = os.path.join(pkg_share, 'rviz', 'go2.rviz')

    # URDF 파일을 문자열로 읽어 robot_description 파라미터로 전달
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    use_gui = LaunchConfiguration('use_gui')
    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_gui', default_value='true',
            description='joint_state_publisher_gui 로 관절을 슬라이더로 움직일지 여부'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='RViz2 를 함께 띄울지 여부'),

        # 1) robot_state_publisher : URDF 기반으로 base -> radar, base -> imu,
        #    base -> 각 다리 링크 TF 를 발행
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        # 2-a) GUI 모드 : 슬라이더로 관절 각도를 직접 조작 (단독 시각화 테스트용)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            condition=IfCondition(use_gui),
        ),

        # 2-b) 비-GUI 모드 : 실제 로봇의 /joint_states 를 쓸 때는 이 노드를 씀
        #      (실로봇 연동 시에는 use_gui:=false 로 두고, 로봇이 /joint_states 를 발행)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            condition=UnlessCondition(use_gui),
        ),

        # 3) RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_path],
            condition=IfCondition(use_rviz),
        ),
    ])
