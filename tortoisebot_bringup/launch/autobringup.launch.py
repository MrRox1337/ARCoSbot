# Author: Aman Mishra
# MISIS: M00983641
# CST3990 UG Project
# Description: This launch file is used to bring up the Tortoisebot in Gazebo simulation with RViz, mapping, and Navigation stack.
# It can also be used to bring up the Tortoisebot in real world.

import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression,Command
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable,IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
import launch_ros
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
  pkg_share = launch_ros.substitutions.FindPackageShare(package='tortoisebot_description').find('tortoisebot_description')
  navigation_dir = os.path.join(get_package_share_directory('tortoisebot_navigation'), 'launch')
  rviz_launch_dir=os.path.join(get_package_share_directory('tortoisebot_description'), 'launch')
  gazebo_launch_dir=os.path.join(get_package_share_directory('tortoisebot_gazebo'), 'launch')
  ydlidar_launch_dir=os.path.join(get_package_share_directory('rplidar_ros'), 'launch')
  #camera_launch_dir=os.path.join(get_package_share_directory('v4l2_camera'), 'launch')
  cartographer_launch_dir=os.path.join(get_package_share_directory('tortoisebot_slam'), 'launch')
  prefix_address = get_package_share_directory('tortoisebot_navigation') 
  default_model_path = os.path.join(pkg_share, 'models/urdf/tortoisebot_simple.xacro')
  default_rviz_config_path = os.path.join(get_package_share_directory('tortoisebot_description'), 'rviz/tortoisebot_sensor_display.rviz')
    
  nav2_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
  params_file_sim = os.path.join(prefix_address, 'config', 'nav2_params_simulation.yaml')
  params_file_robot = os.path.join(prefix_address, 'config', 'nav2_params_robot.yaml')
  
  map_file=LaunchConfiguration('map')
  map_directory = os.path.join(get_package_share_directory(
        'tortoisebot_bringup'), 'maps','room2.yaml')
  use_sim_time=LaunchConfiguration('use_sim_time')
  exploration=LaunchConfiguration('exploration')   
  
  rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        parameters= [{'use_sim_time': use_sim_time}],

    )

  state_publisher_launch_cmd=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rviz_launch_dir, 'state_publisher.launch.py')),
            launch_arguments={'use_sim_time':use_sim_time}.items())

  gazebo_launch_cmd=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_launch_dir, 'gazebo.launch.py')),
            condition=IfCondition(use_sim_time),
            launch_arguments={'use_sim_time':use_sim_time}.items())

  navigation_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_dir, 'navigation.launch.py')),
        launch_arguments={'params_file': params_file_robot}.items())
  
  cartographer_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(cartographer_launch_dir, 'cartographer.launch.py')),
        launch_arguments={'params_file': params_file_robot,
                          'exploration': exploration,
                          'use_sim_time': use_sim_time}.items())  

  ydlidar_launch_cmd=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ydlidar_launch_dir, 'rplidar_a1_launch.py')),
            condition=IfCondition(PythonExpression(['not ', use_sim_time])),
            launch_arguments={'use_sim_time':use_sim_time}.items())
  
  differential_drive_node = Node(
        package='tortoisebot_firmware',
        condition=IfCondition(PythonExpression(['not ', use_sim_time])),
        executable='differential.py',
        name ='differential_drive_publisher',
    )
#  camera_drive_node = Node(
#        package='v4l2_camera',
#        condition=IfCondition(PythonExpression(['not ', use_sim_time])),
#        executable='v4l2_camera_node',
#        name ='camera_publisher',
#    )
#  camera_node = Node(
#      package='camera_ros',
#      condition=IfCondition(PythonExpression(['not ', use_sim_time])),
#      executable='camera_node',
#      name ='pi_camera',
#      parameters= [{'height': 360},{'width': 480}],
#    )
  robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time},{'robot_description': ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),value_type=str)}]
    )

  joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters= [{'use_sim_time': use_sim_time}],
    )
  return LaunchDescription([

    SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
    launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='False',
                                            description='Flag to enable use_sim_time'),
    launch.actions.DeclareLaunchArgument(name='exploration', default_value='True',
                                            description='Flag to enable use_sim_time'),
    launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                          description='Absolute path to robot urdf file'),
    launch.actions.DeclareLaunchArgument(name='map',default_value=map_directory,
                                          description='Map to be used'),
    launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
    # launch.actions.IncludeLaunchDescription(
    #   PythonLaunchDescriptionSource([nav2_launch_dir, '/bringup_launch.py']),
    #   condition=IfCondition(PythonExpression(['not ', exploration])),
    #   launch_arguments={'map': map_directory,
    #                     'use_sim_time': use_sim_time,
    #                     'params_file': params_file_sim}.items(),
    # ),
    Node(
        package='nav2_map_server',
        condition=IfCondition(PythonExpression(['not ', exploration])),
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'yaml_filename': map_file}
                    ]),
    Node(
        package='nav2_lifecycle_manager',
        condition=IfCondition(PythonExpression(['not ', exploration])),
        executable='lifecycle_manager',
        name='lifecycle_manager_mapper',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': ['map_server']}]),

    rviz_node,
    state_publisher_launch_cmd,
    robot_state_publisher_node,
    joint_state_publisher_node,
    ydlidar_launch_cmd,
    differential_drive_node,
#    camera_drive_node,
    gazebo_launch_cmd,
    navigation_launch_cmd, 
    cartographer_launch_cmd

  ]
)

