from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
import os

os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"

def generate_launch_description():

    # arg
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error, fatal).'
    )
    
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace for both arms (optional).'
    )

    # per-arm namespace (used for remapping targets)
    left_namespace_arg = DeclareLaunchArgument(
        'left_namespace',
        default_value='left',
        description='Namespace/prefix for left arm topics.'
    )

    right_namespace_arg = DeclareLaunchArgument(
        'right_namespace',
        default_value='right',
        description='Namespace/prefix for right arm topics.'
    )

    # can ports for each arm
    left_can_port_arg = DeclareLaunchArgument(
        'left_can_port',
        default_value='can0',
        description='CAN port for left arm.'
    )

    right_can_port_arg = DeclareLaunchArgument(
        'right_can_port',
        default_value='can1',
        description='CAN port for right arm.'
    )

    arm_type_arg = DeclareLaunchArgument(
        'arm_type',
        default_value='piper',
        choices=['nero', 'piper', 'piper_h', 'piper_l', 'piper_x'],
        description='Robotic arm type (e.g. nero, piper, piper_h, piper_l, piper_x).'
    )

    # 修改为左右独立的夹爪类型参数
    left_effector_type_arg = DeclareLaunchArgument(
        'left_effector_type',
        default_value='none',
        choices=['none', 'agx_gripper', 'revo2'],
        description='Left arm end effector type (e.g. agx_gripper, revo2).'
    )

    right_effector_type_arg = DeclareLaunchArgument(
        'right_effector_type',
        default_value='none',
        choices=['none', 'agx_gripper', 'revo2'],
        description='Right arm end effector type (e.g. agx_gripper, revo2).'
    )

    auto_enable_arg = DeclareLaunchArgument(
        'auto_enable',
        default_value='true',
        choices=['true', 'false'],
        description='Automatically enable the AGX Arm node.'
    )

    fast_mode_arg = DeclareLaunchArgument(
        'fast_mode',
        default_value='false',
        choices=['true', 'false'],
        description='Enable fast mode for the AGX Arm node.'
    )

    speed_percent_arg = DeclareLaunchArgument(
        'speed_percent',
        default_value='100',
        description='Movement speed as a percentage of maximum speed.'
    )

    pub_rate_arg = DeclareLaunchArgument(
        'pub_rate',
        default_value='200',
        description='Publishing rate for the AGX Arm node.'
    )

    enable_timeout_arg = DeclareLaunchArgument(
        'enable_timeout',
        default_value='5.0',
        description='Timeout in seconds for arm enable/disable operations.'
    )

    tcp_offset_arg = DeclareLaunchArgument(
        'tcp_offset',
        default_value='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]',
        description='TCP offset in x, y, z, roll, pitch, yaw in meters/radians.'
    )

    gripper_default_effort_arg = DeclareLaunchArgument(
        'gripper_default_effort',
        default_value='1.0',
        description='Default effort for gripper commands (>= 0.0).'
    )

    publish_gripper_joint_arg = DeclareLaunchArgument(
        'publish_gripper_joint',
        default_value='true',
        choices=['true', 'false'],
        description='Publish "gripper" (opening width) joint in /feedback/joint_states. '
                    'Set false when used with MoveIt (URDF only has gripper_joint1/2).',
    )

    # helper to create remappings that prepend a per-arm prefix (namespace)
    def arm_remappings(ns_launch_config_name):
        topics = [
            # feedback topics
            'feedback/joint_states',
            'feedback/tcp_pose',
            'feedback/arm_status',
            'feedback/leader_joint_angles',
            'feedback/gripper_status',
            'feedback/hand_status',
            # control topics
            'control/joint_states',
            'control/move_j',
            'control/move_p',
            'control/move_l',
            'control/move_c',
            'control/move_js',
            'control/move_mit',
            'control/hand',
            'control/hand_position_time',
            # services
            'enable_agx_arm',
            'move_home',
            'emergency_stop',
            'exit_teach_mode',
        ]
        remaps = []
        for t in topics:
            # 使用PathJoinSubstitution来正确拼接路径
            remaps.append((t, PathJoinSubstitution([
                LaunchConfiguration('namespace'),
                LaunchConfiguration(ns_launch_config_name),
                t
            ])))
        return remaps

    # left arm node (uses left_can_port and left_effector_type)
    agx_arm_left_node = Node(
        package='agx_arm_ctrl',
        executable='agx_arm_ctrl_single',
        name='agx_arm_left_node',
        namespace=LaunchConfiguration('namespace'),  # optional top-level namespace
        output='screen',
        ros_arguments=['--log-level', LaunchConfiguration('log_level')],
        parameters=[{
            'can_port': LaunchConfiguration('left_can_port'),
            'pub_rate': LaunchConfiguration('pub_rate'),
            'auto_enable': LaunchConfiguration('auto_enable'),
            'fast_mode': LaunchConfiguration('fast_mode'),
            'arm_type': LaunchConfiguration('arm_type'),
            'speed_percent': LaunchConfiguration('speed_percent'),
            'enable_timeout': LaunchConfiguration('enable_timeout'),
            'effector_type': LaunchConfiguration('left_effector_type'),  # 使用左臂夹爪类型
            'tcp_offset': LaunchConfiguration('tcp_offset'),
            'gripper_default_effort': LaunchConfiguration('gripper_default_effort'),
            'publish_gripper_joint': LaunchConfiguration('publish_gripper_joint'),
        }],
        remappings=arm_remappings('left_namespace'),
    )

    # right arm node (uses right_can_port and right_effector_type)
    agx_arm_right_node = Node(
        package='agx_arm_ctrl',
        executable='agx_arm_ctrl_single',
        name='agx_arm_right_node',
        namespace=LaunchConfiguration('namespace'),  # optional top-level namespace
        output='screen',
        ros_arguments=['--log-level', LaunchConfiguration('log_level')],
        parameters=[{
            'can_port': LaunchConfiguration('right_can_port'),
            'pub_rate': LaunchConfiguration('pub_rate'),
            'auto_enable': LaunchConfiguration('auto_enable'),
            'fast_mode': LaunchConfiguration('fast_mode'),
            'arm_type': LaunchConfiguration('arm_type'),
            'speed_percent': LaunchConfiguration('speed_percent'),
            'enable_timeout': LaunchConfiguration('enable_timeout'),
            'effector_type': LaunchConfiguration('right_effector_type'),  # 使用右臂夹爪类型
            'tcp_offset': LaunchConfiguration('tcp_offset'),
            'gripper_default_effort': LaunchConfiguration('gripper_default_effort'),
            'publish_gripper_joint': LaunchConfiguration('publish_gripper_joint'),
        }],
        remappings=arm_remappings('right_namespace'),
    )

    return LaunchDescription([
        # arguments
        log_level_arg,
        namespace_arg,
        left_namespace_arg,
        right_namespace_arg,
        left_can_port_arg,
        right_can_port_arg,
        arm_type_arg,
        left_effector_type_arg,    # 左臂夹爪参数
        right_effector_type_arg,   # 右臂夹爪参数
        auto_enable_arg,
        fast_mode_arg,
        speed_percent_arg,
        pub_rate_arg,
        enable_timeout_arg,
        tcp_offset_arg,
        gripper_default_effort_arg,
        publish_gripper_joint_arg,
        # nodes
        agx_arm_left_node,
        agx_arm_right_node,
    ])