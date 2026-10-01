from launch import LaunchDescription
from launch.actions import GroupAction, DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# 값을 줬을 때만 go2_fix.yaml 뒤에 덮어쓰는 선택 인자 (A/B 배치용). 비워 두면 yaml 값 그대로.
#   예: ros2 launch point_lio mapping_go2_fix.launch.py leg_en:=false
OVERRIDE_BOOL_ARGS = ('leg_en', 'zupt_en')


def _bool_overrides(context):
    overrides = {}
    for name in OVERRIDE_BOOL_ARGS:
        val = LaunchConfiguration(name).perform(context).strip().lower()
        if not val:
            continue
        if val not in ('true', 'false'):
            raise RuntimeError(f"[point_lio] {name}:={val} - 'true' 또는 'false' 만 받습니다")
        overrides[name] = (val == 'true')
    return overrides


def laser_mapping_setup(context, *args, **kwargs):
    # Node parameters, including those from the YAML configuration file
    laser_mapping_params = [
        PathJoinSubstitution([
            FindPackageShare('point_lio'),
            'config', 'go2_fix.yaml'
        ])
    ]
    overrides = _bool_overrides(context)
    if overrides:
        laser_mapping_params.append(overrides)   # 뒤에 온 값이 yaml 을 덮어씀

    # Node definition for laserMapping with Point-LIO
    laser_mapping_node = Node(
        package='point_lio',
        executable='pointlio_mapping',
        name='laserMapping',
        output='screen',
        parameters=laser_mapping_params,
        # prefix='gdb -ex run --args'
    )
    return [laser_mapping_node]


def generate_launch_description():
    # Declare the RViz argument
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Flag to launch RViz.')

    override_args = [
        DeclareLaunchArgument(
            name, default_value='',
            description=f"Override {name} in go2_fix.yaml ('true'/'false'). Empty keeps the yaml value.")
        for name in OVERRIDE_BOOL_ARGS
    ]

    # Conditional RViz node launch
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('point_lio'),
            'rviz_cfg', 'loam_livox.rviz'
        ])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        prefix='nice'
    )

    # Assemble the launch description
    ld = LaunchDescription([
        rviz_arg,
        *override_args,
        OpaqueFunction(function=laser_mapping_setup),
        GroupAction(
            actions=[rviz_node],
            condition=IfCondition(LaunchConfiguration('rviz'))
        ),
    ])

    return ld
