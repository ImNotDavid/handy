import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
 
def generate_launch_description():
 
  # Set the path to this package.
  pkg_share = FindPackageShare(package='handy_ros2').find('handy_ros2')
 
  # Set the path to the URDF file
  default_urdf_model_path = os.path.join(pkg_share, 'urdf/robot.urdf')
  print(default_urdf_model_path)
 
  ########### YOU DO NOT NEED TO CHANGE ANYTHING BELOW THIS LINE ##############  
  # Launch configuration variables specific to simulation
  urdf_model = LaunchConfiguration('urdf_model')
  use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
  use_joint_state_pub = LaunchConfiguration('use_joint_state_pub')
 
  # Declare the launch arguments  
  declare_urdf_model_path_cmd = DeclareLaunchArgument(
    name='urdf_model', 
    default_value=default_urdf_model_path, 
    description='Absolute path to robot urdf file')
   
  declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
    name='use_robot_state_pub',
    default_value='True',
    description='Whether to start the robot state publisher')
  
  declare_use_joint_state_pub_cmd = DeclareLaunchArgument(
    name='use_joint_state_pub',
    default_value='False',
    description='Whether to start the robot joint publisher')
    
  # Specify the actions
 
  # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
  start_robot_state_publisher_cmd = Node(
    condition=IfCondition(use_robot_state_pub),
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'use_sim_time': False, 
    'robot_description': Command(['xacro ', urdf_model])}],
    arguments=[default_urdf_model_path])
  
  start_joint_state_publisher_cmd = Node(
    condition=IfCondition(use_joint_state_pub),
    package='joint_state_publisher',
    executable='joint_state_publisher',
    name='joint_state_publisher',
    arguments=[default_urdf_model_path])
   
  # Create the launch description and populate
  ld = LaunchDescription()
 
  # Declare the launch options
  ld.add_action(declare_use_joint_state_pub_cmd)  
  ld.add_action(start_joint_state_publisher_cmd)
  ld.add_action(declare_urdf_model_path_cmd)
  ld.add_action(declare_use_robot_state_pub_cmd)  

  # Add any actions
  ld.add_action(start_robot_state_publisher_cmd)
 
  return ld