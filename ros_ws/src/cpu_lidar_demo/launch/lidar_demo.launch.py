import os

from typing import List

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    lidar_type = LaunchConfiguration('lidar_type').perform(context)
    render_mode = LaunchConfiguration('render_mode').perform(context)
    use_gz_gui = LaunchConfiguration('gz_gui').perform(context)
    use_rviz = LaunchConfiguration('rviz').perform(context)
    container_mode = LaunchConfiguration('container_mode').perform(context)
    hard_headless = LaunchConfiguration('hard_headless').perform(context)
    lidar_3d = LaunchConfiguration('lidar_3d').perform(context)

    pkg_dir = get_package_share_directory('cpu_lidar_demo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Determine world file based on lidar_type, hard_headless, and lidar_3d
    world_name = ''
    if hard_headless == 'true':
        if lidar_type == 'cpu':
            world_name = 'cpu_lidar_hard_headless'
        else:
            world_name = 'gpu_lidar_hard_headless'
    else:
        if lidar_type == 'cpu':
            world_name = 'cpu_lidar_demo'
        else:
            world_name = 'gpu_lidar_demo'

    if lidar_3d == 'true':
        world_name += '_3d'

    world_file = os.path.join(pkg_dir, 'worlds', f'{world_name}.sdf')

    if lidar_type == 'cpu':
        gz_topic = '/cpu_lidar'
        sensor_frame = 'lidar_platform/link/cpu_lidar'
    else:
        gz_topic = '/gpu_lidar'
        sensor_frame = 'gpu_lidar_platform/link/gpu_lidar'

    # Force disable GUI in hard_headless mode
    if hard_headless == 'true':
        use_gz_gui = 'false'

    gz_sim = Node(
        package='ros_gz_sim',
        executable='gzserver',
        parameters=[{'world_sdf_file': world_file}],
        arguments=['-r'],
        output='screen',
    )

    # Merge current environment with overrides
    def get_env(overrides):
        env = dict(os.environ)
        env.update(overrides)
        return env

    sw_render_env = {
        'LIBGL_ALWAYS_SOFTWARE': '1',
        'QT_QUICK_BACKEND': 'software',
        '__GLX_VENDOR_LIBRARY_NAME': 'mesa',
        '__EGL_VENDOR_LIBRARY_NAME': 'mesa'
    }

    hw_render_env = {
        '__GLX_VENDOR_LIBRARY_NAME': 'nvidia',
        '__NV_PRIME_RENDER_OFFLOAD': '1',
        '__VK_LAYER_NV_optimus': 'NVIDIA_only',
    }

    gz_client_env = sw_render_env if render_mode == 'software' else hw_render_env

    gz_client = ExecuteProcess(
        cmd=['/home/alexis/gazebo_ws/install/libexec/gz/sim10/gz-sim-main', '-g'],
        additional_env=get_env(gz_client_env),  # type: ignore[arg-type]
        output='screen',
    )

    bridge_scan = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_scan',
        arguments=[
            gz_topic + '@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        remappings=[(gz_topic, '/scan')],
        output='screen',
    )

    bridge_points = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_points',
        arguments=[
            gz_topic + '/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
        remappings=[(gz_topic + '/points', '/points')],
        output='screen',
    )

    # Static transform from world to lidar_frame (fixed reference)
    static_tf_world = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_world',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'lidar_frame'],
        output='screen',
    )

    # Static transform from lidar_frame to actual sensor frame
    static_tf_sensor = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_sensor',
        arguments=['0.05', '0.05', '0.05', '0', '0', '0', 'lidar_frame', sensor_frame],
        output='screen',
    )

    rviz_config = os.path.join(pkg_dir, 'rviz', 'lidar.rviz')
    # Prepend OGRE path to LD_LIBRARY_PATH for rviz2
    ogre_path = '/opt/ros/jazzy/opt/rviz_ogre_vendor/lib'
    current_ld_path = os.environ.get('LD_LIBRARY_PATH', '')
    new_ld_path = f"{ogre_path}:{current_ld_path}" if current_ld_path else ogre_path

    rviz_env = dict(sw_render_env) if render_mode == 'software' else dict(hw_render_env)
    rviz_env['LD_LIBRARY_PATH'] = new_ld_path

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        env=get_env(rviz_env),
        output='screen',
    )

    nodes: List[Node | ExecuteProcess] = []

    # Server mode: only launch gz_sim (gzserver)
    if container_mode == 'server':
        nodes.append(gz_sim)
    # UI mode: launch everything except gz_sim (bridges, tf, optionally gz_client and rviz)
    elif container_mode == 'ui':
        nodes.extend([bridge_scan, bridge_points, static_tf_world, static_tf_sensor])
        if use_gz_gui == 'true':
            nodes.append(gz_client)
        if use_rviz == 'true':
            nodes.append(rviz)
    # All mode: launch everything
    else:
        nodes.extend([gz_sim, bridge_scan, bridge_points, static_tf_world, static_tf_sensor])
        if use_gz_gui == 'true':
            nodes.append(gz_client)
        if use_rviz == 'true':
            nodes.append(rviz)

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'lidar_type',
            default_value='cpu',
            choices=['cpu', 'gpu'],
            description='Lidar implementation: "cpu" (physics raycast) or "gpu" (rendering)',
        ),
        DeclareLaunchArgument(
            'render_mode',
            default_value='hardware',
            choices=['hardware', 'software'],
            description='GUI rendering mode for Gazebo / RViz',
        ),
        DeclareLaunchArgument(
            'gz_gui',
            default_value='true',
            choices=['true', 'false'],
            description='Run Gazebo GUI (gz_client)',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            choices=['true', 'false'],
            description='Run RViz visualization',
        ),
        DeclareLaunchArgument(
            'container_mode',
            default_value='all',
            choices=['all', 'server', 'ui'],
            description='Launch mode: "all" (server + UI), "server" (gzserver only), "ui" (bridges + GUI tools)',
        ),
        DeclareLaunchArgument(
            'hard_headless',
            default_value='false',
            choices=['true', 'false'],
            description='Hard headless mode: forces gz_gui:=false to disable GPU rendering, but allows RViz to run. '
                        'Useful to demonstrate CPU lidar works in RViz while GPU lidar does not.',
        ),
        DeclareLaunchArgument(
            'lidar_3d',
            default_value='false',
            choices=['true', 'false'],
            description='Use 3D Lidar world (16 beams vertical)',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
