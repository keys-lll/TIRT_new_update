import os
import copy
import math
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, LogInfo, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = share('mecanum_navigation')
    # geometry2 0.25.24 fixes the LaserScan MessageFilter / TF listener lock inversion.
    # Keep the tested core library scoped to this launch and its child processes.
    tf2_fix_dir = os.path.join(os.path.expanduser(os.environ.get('TIRT_WORKSPACE', '~/mecanum_ros2')), 'tf2_0.25.24', 'lib')
    if not os.path.isfile(os.path.join(tf2_fix_dir, 'libtf2.so')):
        raise RuntimeError('Missing tested tf2 0.25.24 library: ' + tf2_fix_dir)
    tf2_library_path = tf2_fix_dir + os.pathsep + os.environ.get('LD_LIBRARY_PATH', '')
    def include(package, filename, arguments=None, condition=None):
        options = {'launch_arguments': (arguments or {}).items()}
        if condition is not None:
            options['condition'] = condition
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share(package), 'launch', filename)), **options)
    actions = [
        SetEnvironmentVariable(name='LD_LIBRARY_PATH', value=tf2_library_path),
        LogInfo(msg='Navigation TF fix: loading tested tf2 0.25.24 from ' + tf2_fix_dir),
        DeclareLaunchArgument('start_hardware', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_keepout', default_value='false'),
        DeclareLaunchArgument('keepout_mask', default_value=os.path.join(os.path.expanduser(os.environ.get('TIRT_WORKSPACE', '~/mecanum_ros2')), 'maps/keepout.yaml')),
        DeclareLaunchArgument('initial_waypoint', default_value='', description='Known physical starting waypoint; empty uses manual localization'),
        DeclareLaunchArgument('waypoint_file', default_value=os.path.join(os.path.expanduser(os.environ.get('TIRT_WORKSPACE', '~/mecanum_ros2')), 'src/car_waypoints/config/waypoints.yaml')),
        DeclareLaunchArgument('stm32_port', default_value='/dev/serial/by-id/usb-1a86_USB_Single_Serial_597B018072-if00'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/serial/by-id/usb-1a86_USB_Single_Serial_5893011986-if00'),
        DeclareLaunchArgument('use_lidar', default_value='true'),
        include('mecanum_navigation', 'hardware.launch.py',
                {**{key: LaunchConfiguration(key) for key in ('stm32_port', 'lidar_port', 'use_lidar')},
                 'cmd_vel_topic': '/cmd_vel', 'require_fresh_scan': 'true'},
                IfCondition(LaunchConfiguration('start_hardware'))),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', os.path.join(pkg, 'config', 'map_tf.rviz')],
             condition=IfCondition(LaunchConfiguration('use_rviz'))),
    ]
    actions.append(DeclareLaunchArgument('map', description='Absolute path to a saved map YAML (required)'))
    def start_navigation(context):
        params_file = os.path.join(pkg, 'config', 'nav2_params.yaml')
        with open(params_file) as stream:
            config = yaml.safe_load(stream)
        controller = config['controller_server']['ros__parameters']
        prep_settings = copy.deepcopy(controller['FollowPath'])
        prep_settings.update(controller['FollowPathPrep'])
        controller['FollowPathPrep'] = prep_settings
        transit_settings = copy.deepcopy(controller['FollowPath'])
        transit_settings.update(controller['FollowPathTransit'])
        controller['FollowPathTransit'] = transit_settings
        bt = config['bt_navigator']['ros__parameters']
        bt['default_nav_to_pose_bt_xml'] = os.path.join(pkg, 'config', 'navigate_default.xml')
        bt['default_nav_through_poses_bt_xml'] = os.path.join(pkg, 'config', 'navigate_through_default.xml')
        name = LaunchConfiguration('initial_waypoint').perform(context)
        messages = []
        if name:
            filename = os.path.expanduser(LaunchConfiguration('waypoint_file').perform(context))
            with open(filename) as stream:
                points = yaml.safe_load(stream)['waypoints']
            if name not in points:
                raise RuntimeError(f'Initial waypoint {name!r} not found in {filename}')
            point = points[name]
            pose = {key: float(point[key]) for key in ('x', 'y', 'yaw')}
            if not all(math.isfinite(value) for value in pose.values()):
                raise RuntimeError('Initial waypoint contains nonfinite coordinates')
            amcl = config['amcl']['ros__parameters']
            amcl['set_initial_pose'] = True
            amcl['always_reset_initial_pose'] = True
            amcl['initial_pose'] = dict(pose, z=0.0)
            messages.append(LogInfo(msg=f'Initial localization from {name}: {pose}. Robot must physically start at this pose.'))
        keepout_nodes = []
        if LaunchConfiguration('use_keepout').perform(context).lower() in ('true', '1', 'yes'):
            mask_file = os.path.expanduser(LaunchConfiguration('keepout_mask').perform(context))
            if not os.path.isfile(mask_file):
                raise RuntimeError('Keepout mask YAML not found: ' + mask_file)
            keepout_nodes = [
                Node(package='nav2_map_server', executable='map_server',
                     name='keepout_mask_server', output='screen', parameters=[{
                         'use_sim_time': False, 'yaml_filename': mask_file,
                         'topic_name': '/keepout_filter_mask', 'frame_id': 'map'}]),
                Node(package='nav2_map_server', executable='costmap_filter_info_server',
                     name='keepout_filter_info_server', output='screen', parameters=[{
                         'use_sim_time': False, 'type': 0,
                         'filter_info_topic': '/costmap_filter_info',
                         'mask_topic': '/keepout_filter_mask',
                         'base': 0.0, 'multiplier': 1.0}]),
                Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                     name='lifecycle_manager_keepout', output='screen', parameters=[{
                         'use_sim_time': False, 'autostart': True,
                         'node_names': ['keepout_mask_server', 'keepout_filter_info_server']}]),
            ]
            messages.append(LogInfo(msg='Keepout mask: ' + mask_file))
        else:
            for name in ('global_costmap', 'local_costmap'):
                config[name][name]['ros__parameters'].pop('filters', None)
        with tempfile.NamedTemporaryFile(mode='w', prefix='mecanum_nav_runtime_', suffix='.yaml', delete=False) as stream:
            yaml.safe_dump(config, stream, sort_keys=False)
            params_file = stream.name
        return messages + keepout_nodes + [include('nav2_bringup', 'bringup_launch.py', {
            'map': LaunchConfiguration('map'), 'slam': 'False', 'use_sim_time': 'false',
            'params_file': params_file, 'autostart': 'true', 'use_composition': 'False',
        })]
    actions.append(OpaqueFunction(function=start_navigation))
    return LaunchDescription(actions)
