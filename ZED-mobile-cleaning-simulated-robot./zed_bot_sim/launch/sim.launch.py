import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
  DeclareLaunchArgument,
  ExecuteProcess,
  IncludeLaunchDescription,
  RegisterEventHandler,
  SetEnvironmentVariable)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit

from nav2_common.launch import ReplaceString

 
def generate_launch_description():
  # Set the path to this package.
  description_pkg_path = get_package_share_directory('zed_bot_description')
  rviz_pkg_path = get_package_share_directory('zed_bot_rviz')
  sim_pkg_path = get_package_share_directory('zed_bot_sim') 

  # initial robot pose
  x_pos = 0.0; y_pos = 0.0; z_pos = 0.5; yaw = 0.0

  # Set the path to the world file
  world_file_name = 'empty.sdf'
  world_file_path = os.path.join(sim_pkg_path, 'world', world_file_name)

  # Set rviz config file
  rviz_file_name = 'sim.rviz'
  rviz_file_path = os.path.join(rviz_pkg_path, 'rviz', rviz_file_name)
 

  # set some ignition environment variable
  gz_models_path = os.path.join(sim_pkg_path, "model")
  gz_sim_system_plugin_path = '/opt/ros/humble/lib/'

  set_env_ign_resource_cmd = SetEnvironmentVariable(
          name="IGN_GAZEBO_RESOURCE_PATH",
          value=gz_models_path,
      )
  set_env_ign_model_cmd = SetEnvironmentVariable(
          name="IGN_GAZEBO_MODEL_PATH",
          value=gz_models_path,
      )
  
  set_env_ign_path_cmd = SetEnvironmentVariable(
          name="GZ_SIM_SYSTEM_PLUGIN_PATH",
          value=gz_sim_system_plugin_path,
      )
  #---------------------------------------------------
 

  # Launch configuration variables specific to simulation
  headless = LaunchConfiguration('headless')
  use_sim_time = LaunchConfiguration('use_sim_time')
  world_path = LaunchConfiguration('world_path')
  rviz_path = LaunchConfiguration('rviz_path')
  use_rviz = LaunchConfiguration('use_rviz')
  gz_verbosity = LaunchConfiguration('gz_verbosity')
  robot_name = LaunchConfiguration('robot_name')
 
  declare_headless_cmd = DeclareLaunchArgument(
    name='headless',
    default_value='False',
    description='Whether to run only gzserver')
     
  declare_use_sim_time_cmd = DeclareLaunchArgument(
    name='use_sim_time',
    default_value='True',
    description='Use simulation (Gazebo) clock if true')
 
  declare_world_path_cmd = DeclareLaunchArgument(
    name='world_path',
    default_value=world_file_path,
    description='Full path to the world model file to load')
  
  declare_rviz_path_cmd = DeclareLaunchArgument(
    name='rviz_path',
    default_value=rviz_file_path,
    description='Full path to the world model file to load')
  
  declare_use_rviz_cmd = DeclareLaunchArgument(
    'use_rviz',
    default_value= 'True',
    description='whether to run sim with rviz or not')
  
  declare_gz_verbosity_cmd = DeclareLaunchArgument(
    'gz_verbosity',
    default_value= '3',
    description='Verbosity level for Ignition Gazebo (0~4).')
  
  declare_robot_name_cmd = DeclareLaunchArgument(
      name='robot_name',
      default_value='zed_bot',
      description='name of the robot')
  #------------------------------------------------------------


  # Specify the actions
  rsp_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
          [os.path.join(description_pkg_path,'launch','rsp.launch.py')]
      ), 
      launch_arguments={'use_sim_time': use_sim_time,
                        'run_gz_sim': 'True'}.items()
  )

  rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_path],
        output='screen',
        condition=IfCondition(use_rviz)
  )


  start_ign_gazebo = ExecuteProcess(
      condition=UnlessCondition(headless),
      cmd=['ign', 'gazebo',  '-r', '-v', gz_verbosity, world_path],
      output='screen',
      # shell=False,
  )
        
  
  start_ign_gazebo_headless = ExecuteProcess(
      condition=IfCondition(headless),
      cmd=['ign', 'gazebo',  '-r', '-v', gz_verbosity, '-s', '--headless-rendering', world_path],
      output='screen',
      # shell=False,
  )
        

  bridge_config_file_path = os.path.join(sim_pkg_path, 'config', 'gz_bridge_config.yaml')
  # A <entity> placeholder is used in the bridge config file to be replaced by the entity name.
  bridge_config = ReplaceString(
      source_file=bridge_config_file_path,
      replacements={'<entity>': robot_name},
  )

  bridge_node = Node(
      package='ros_gz_bridge',
      executable='parameter_bridge',
      output='screen',
      parameters=[{
          'config_file': bridge_config
      }],
  )
  
  spawn_entity_in_ign = Node(
      package='ros_gz_sim',
      executable='create',
      output='screen',
      arguments=[
          '-topic', 'robot_description', 
          '-name', robot_name,
          '-x', str(x_pos),
          '-y', str(y_pos),
          '-z', str(z_pos),
          '-Y', str(yaw),
          ],
      parameters=[{"use_sim_time": use_sim_time}]
  )
  
  # Create the launch description
  ld = LaunchDescription()

  ld.add_action(set_env_ign_resource_cmd)
  ld.add_action(set_env_ign_model_cmd)
  ld.add_action(set_env_ign_path_cmd)
 
  # add the necessary declared launch arguments to the launch description
  ld.add_action(declare_headless_cmd)
  ld.add_action(declare_use_sim_time_cmd)
  ld.add_action(declare_world_path_cmd)
  ld.add_action(declare_rviz_path_cmd)
  ld.add_action(declare_use_rviz_cmd)
  ld.add_action(declare_gz_verbosity_cmd)
  ld.add_action(declare_robot_name_cmd)
 
  # Add the nodes to the launch description
  ld.add_action(rsp_launch)
  ld.add_action(rviz_node)
  ld.add_action(start_ign_gazebo)
  ld.add_action(start_ign_gazebo_headless)
  ld.add_action(bridge_node)
  ld.add_action(spawn_entity_in_ign)

 
  return ld