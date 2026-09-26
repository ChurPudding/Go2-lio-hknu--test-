#!/usr/bin/env python3
# ============================================================
# Point-LIO 통합 런치 파일  (catkin_point_lio_unilidar 기준)
#   대체 대상:
#     mapping_*.launch.py (센서/프로파일별)  → lidar:=<key>
#     correct_odom_*.launch.py               → odom_only:=true
#     gdb_debug_example.launch.py            → debug:=true
#
# 사용 예:
#   ros2 launch point_lio point_lio.launch.py                      # go2_fix, rviz on (기본)
#   ros2 launch point_lio point_lio.launch.py lidar:=go2_raw
#   ros2 launch point_lio point_lio.launch.py lidar:=l1 rviz:=false
#   ros2 launch point_lio point_lio.launch.py lidar:=go2_fix odom_only:=true
#   ros2 launch point_lio point_lio.launch.py lidar:=go2_fix debug:=true
#
# ※ 패키지명(point_lio)과 config yaml 파일명이 본인 것과 같은지 확인:
#    ros2 pkg list | grep -i lio   /   ls ~/catkin_point_lio_unilidar/src/point_lio_ros2/config
# ============================================================
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# 패키지명 (본인 fork에 맞게: point_lio / point_lio_ros2 등)
PACKAGE = 'point_lio'

# 선택키 → config yaml
LIDAR_CFG = {
    'go2_fix':  'go2_fix.yaml',      # 효신 커스텀 (Go2 전용, 주력)
    'go2_raw':  'go2_raw.yaml',      # 효신 커스텀 (Go2 전용)
    'l1':       'unilidar_l1.yaml',
    'l2':       'unilidar_l2.yaml',
    'avia':     'avia.yaml',
    'mid360':   'mid360.yaml',
    'ouster64': 'ouster64.yaml',
    'velody16': 'velody16.yaml',
    'horizon':  'horizon.yaml',
}


def _is_true(val: str) -> bool:
    return str(val).strip().lower() in ('1', 'true', 'yes', 'on')


def launch_setup(context, *args, **kwargs):
    pkg = FindPackageShare(PACKAGE)

    lidar = LaunchConfiguration('lidar').perform(context).lower()
    if lidar not in LIDAR_CFG:
        raise RuntimeError(
            f"[point_lio] 알 수 없는 lidar:={lidar}. "
            f"가능한 값: {', '.join(LIDAR_CFG)}")
    cfg_yaml = LIDAR_CFG[lidar]

    rviz_on   = _is_true(LaunchConfiguration('rviz').perform(context))
    odom_only = _is_true(LaunchConfiguration('odom_only').perform(context))
    debug     = _is_true(LaunchConfiguration('debug').perform(context))

    # ── 공통 파라미터 (스톡 mapping_*.launch.py와 동일) ──
    params = [
        PathJoinSubstitution([pkg, 'config', cfg_yaml]),
        {
            'use_imu_as_input': False,   # False = 출력모델(ω 상태 존재 → ZUPT/ZARU 가능)
            'prop_at_freq_of_imu': True,
            'check_satu': True,
            'init_map_size': 10,
            'point_filter_num': 1,       # 1 또는 3
            'space_down_sample': True,
            'filter_size_surf': 0.1,
            'filter_size_map': 0.1,
            'cube_side_length': 1000.0,
            'runtime_pos_log_enable': False,
        },
    ]

    # ── odom_only 모드 (correct_odom_*.launch.py 대체): 파라미터 3개 추가 + rviz off ──
    if odom_only:
        params.append({
            'odom_only': True,
            'odom_header_frame_id': LaunchConfiguration('odom_frame').perform(context),
            'odom_child_frame_id':  LaunchConfiguration('base_frame').perform(context),
        })
        rviz_on = False

    # ── laserMapping 노드 (debug=true면 gdb prefix; gdb_debug_example 대체) ──
    node_kwargs = dict(
        package=PACKAGE,
        executable='pointlio_mapping',
        name='laserMapping',
        output='screen',
        parameters=params,
    )
    if debug:
        node_kwargs['prefix'] = 'gdb -ex run --args'

    nodes = [Node(**node_kwargs)]

    if rviz_on:
        nodes.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', PathJoinSubstitution([pkg, 'rviz_cfg', 'loam_livox.rviz'])],
            prefix='nice',
        ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'lidar', default_value='go2_fix',
            description='선택키: ' + ', '.join(LIDAR_CFG)),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='RViz 실행 여부 (odom_only=true면 자동 off)'),
        DeclareLaunchArgument(
            'odom_only', default_value='false',
            description='true면 오도메트리 전용 모드 (correct_odom 대체)'),
        DeclareLaunchArgument(
            'debug', default_value='false',
            description='true면 gdb로 실행 (gdb_debug_example 대체)'),
        DeclareLaunchArgument(
            'odom_frame', default_value='odom',
            description='odom_only 모드의 header frame id'),
        DeclareLaunchArgument(
            'base_frame', default_value='base_link',
            description='odom_only 모드의 child frame id'),
        OpaqueFunction(function=launch_setup),
    ])
