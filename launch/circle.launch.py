from sys import prefix
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_source import LaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch.substitutions import EnvironmentVariable
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # package info
    package_name = "demo_gazebo_ros_car_minimal"
    package_path = FindPackageShare(package_name)
    models_path = PathJoinSubstitution([package_path, "models"])
    urdf_model_path = PathJoinSubstitution([models_path, "car/model.urdf"])
    rviz_config_path = PathJoinSubstitution([package_path, "configs", "rviz2_config.rviz"])
    ign_gazebo_config_path = PathJoinSubstitution([package_path, "configs", "ign_gazebo_gui.config"])
    world_sdf_path = PathJoinSubstitution([models_path, 'world.sdf'])

    # ign gazebo
    ros_ign_gazebo_package_launch_path = os.path.join(
        get_package_share_directory('ros_ign_gazebo'), 'launch/ign_gazebo.launch.py')
    ros_ign_gazebo_launch_arguments = {"ign_args": [world_sdf_path, " --gui-config ", ign_gazebo_config_path,]}.items()
    launch_description_source = PythonLaunchDescriptionSource(ros_ign_gazebo_package_launch_path)
    ros_ign_gazebo_launch_include = IncludeLaunchDescription(launch_description_source,
                                                             launch_arguments=ros_ign_gazebo_launch_arguments)

    # ros ign bridge
    # 将 ROS 消息转换为 Ignition 消息
    ros_ign_bridge_topics = [
        "/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist",
        "/camera@sensor_msgs/msg/Image@ignition.msgs.Image",
        "/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry",
        "/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo",
        "/joint_states@sensor_msgs/msg/JointState@ignition.msgs.Model",
        "/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V"
    ]
    ros_ign_bridge_node = Node(package="ros_ign_bridge", executable="parameter_bridge",
                               arguments=ros_ign_bridge_topics, name="ros_ign_bridge")

    # state_publisher
    # 用于发布 URDF 模型的状态
    state_publisher_node = Node(package="robot_state_publisher", executable="robot_state_publisher",
                                arguments=[urdf_model_path])

    # rviz
    rviz_node = Node(package="rviz2", executable="rviz2", arguments=["-d", rviz_config_path])
    
    # draw circle
    draw_node = Node(package="demo_gazebo_ros_car_minimal", executable="circle", output="screen")

    return LaunchDescription(
        [
            SetEnvironmentVariable(name="SDF_PATH", value=models_path),
            ros_ign_gazebo_launch_include,       
            ros_ign_bridge_node,           # ROS-Ign bridge
            state_publisher_node,   # state publisher
            rviz_node,      # rviz
            draw_node   # draw circle
        ]
    )
