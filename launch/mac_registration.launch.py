# Copyright 2026 Dun Yan Ong. Licensed under the Apache License, Version 2.0.
"""Bring up mac_registration with a D455, a bag, or an already-running perception stack.

Examples
  # D455 + GPU-free tabletop segmentation + MAC + RViz (object mode)
  ros2 launch mac_registration mac_registration.launch.py camera_type:=d455

  # same, from a recording (ours or vision_tool's record_bag.sh)
  ros2 launch mac_registration mac_registration.launch.py camera_type:=bag bag_path:=/path/to/bag

  # the team's YOLO/SAM chain already runs (it starts the camera itself)
  ros2 launch mac_registration mac_registration.launch.py object_source:=yolo_sam label:=bottle

  # scene mode for the localisation team, on their cloud
  ros2 launch mac_registration mac_registration.launch.py mode:=scene input_cloud:=/obstacle_points
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _truthy(value: str) -> bool:
    return value.strip().lower() in ('1', 'true', 'yes', 'on')


def _setup(context, *args, **kwargs):
    get = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    pkg = get_package_share_directory('mac_registration')
    camera_type = get('camera_type')
    mode = get('mode')
    object_source = get('object_source')
    input_cloud = get('input_cloud')
    params_file = get('params_file') or os.path.join(
        pkg, 'config', 'params.yaml' if mode == 'object' else 'params_scene.yaml')
    tabletop_file = get('tabletop_params_file') or os.path.join(pkg, 'config', 'tabletop.yaml')
    fps = get('fps')
    width = get('width')
    height = get('height')
    actions = []

    if camera_type == 'd455':
        rs_launch = os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rs_launch),
            launch_arguments={
                'camera_name': 'camera',
                'camera_namespace': 'camera',
                'device_type': 'd455',
                'enable_color': 'true',
                'enable_depth': 'true',
                'enable_infra1': 'false',
                'enable_infra2': 'false',
                'enable_gyro': 'false',
                'enable_accel': 'false',
                'align_depth.enable': 'true',
                'pointcloud.enable': 'false',
                # The names realsense2_camera 4.57 reads. Under WSL keep both streams at or
                # below 480x270 (USB/IP limit); vision_tool's bringup asks for 848x480 depth and
                # 1280x720 colour, which only stream reliably on native Ubuntu.
                'depth_module.depth_profile': f'{width},{height},{fps}',
                'rgb_camera.color_profile': f'{width},{height},{fps}',
            }.items()))
    elif camera_type == 'bag':
        bag_path = get('bag_path')
        if not bag_path:
            raise RuntimeError('camera_type:=bag needs bag_path:=<directory or .mcap>')
        actions.append(ExecuteProcess(cmd=['ros2', 'bag', 'play', bag_path, '--loop'], output='screen'))
    elif camera_type != 'none':
        raise RuntimeError(f"camera_type must be none, d455 or bag (got '{camera_type}')")

    if input_cloud:
        topic = input_cloud
    elif object_source == 'tabletop':
        topic = '/mac_registration/tabletop_points'
    elif mode == 'scene':
        topic = '/obstacle_points'
    else:
        topic = '/yolo_sam/cleaned_points'

    if object_source == 'tabletop' and not input_cloud:
        actions.append(Node(
            package='mac_registration', executable='tabletop_object_cloud_node',
            name='tabletop_object_cloud', parameters=[tabletop_file], output='screen'))

    overrides = {
        'publish_tf': _truthy(get('publish_tf')),
    }
    if get('label'):
        overrides['label'] = get('label')
    actions.append(Node(
        package='mac_registration', executable='mac_registration_node', name='mac_registration',
        parameters=[params_file, overrides],
        remappings=[('~/input_cloud', topic)],
        # Idle OpenMP threads must sleep, or they spin and starve the ROS executor.
        additional_env={'OMP_WAIT_POLICY': 'PASSIVE'},
        output='screen'))

    if _truthy(get('rviz')):
        actions.append(Node(
            package='rviz2', executable='rviz2', name='mac_registration_rviz',
            arguments=['-d', os.path.join(pkg, 'rviz', 'mac_registration.rviz')], output='log'))
    return actions


def generate_launch_description():
    args = [
        DeclareLaunchArgument('camera_type', default_value='none', description='none | d455 | bag'),
        DeclareLaunchArgument('bag_path', default_value='', description='bag directory or .mcap (camera_type:=bag)'),
        DeclareLaunchArgument('object_source', default_value='tabletop',
                              description='tabletop (GPU-free stand-in) | yolo_sam (/yolo_sam/cleaned_points)'),
        DeclareLaunchArgument('mode', default_value='object', description='object | scene'),
        DeclareLaunchArgument('label', default_value='', description='reference name (empty = from params file)'),
        DeclareLaunchArgument('input_cloud', default_value='', description='explicit input topic override'),
        DeclareLaunchArgument('params_file', default_value='', description='node parameters (empty = profile for mode)'),
        DeclareLaunchArgument('tabletop_params_file', default_value='', description='tabletop node parameters'),
        DeclareLaunchArgument('fps', default_value='15', description='D455 frame rate: 5, 15 or 30'),
        # Under WSL the D455 travels over USB/IP, which drops frames larger than ~480x270
        # (measured 2026-10-05: 480x270@15 -> 15 Hz; 640x360@15 -> 3 Hz; 848x480 -> stalls).
        # On native Ubuntu use width:=848 height:=480.
        DeclareLaunchArgument('width', default_value='480', description='D455 stream width (both streams)'),
        DeclareLaunchArgument('height', default_value='270', description='D455 stream height (both streams)'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('publish_tf', default_value='false', description='broadcast <label>_frame'),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=_setup)])
