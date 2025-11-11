from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Launch argument for showing GUI or not
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz GUI"
        )
    )
    gui = LaunchConfiguration("gui")

    # Get URDF from xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
            PathJoinSubstitution([
                FindPackageShare("fanuc_lrmate200ic_support"),
                "urdf",
                "lrmate200ic.xacro"   # <-- launch main xacro that calls your macro
            ])
        ]
    )
    robot_description = {
    "robot_description": ParameterValue(robot_description_content, value_type=str)
    }
    # rviz_config_file = PathJoinSubstitution(
    #     [FindPackageShare("fanuc_lrmate200ic_support"), "rviz", "view_fanuc.rviz"]
    # )
    
    # rviz_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     output="log",
    #     arguments=["-d", rviz_config_file],
    #     condition=IfCondition(gui),
    # )   

    # Controller parameters
    robot_controllers = PathJoinSubstitution([
        FindPackageShare("fanuc_lrmate200ic_support"),
        "config",
        "controllers.yaml"
    ])

    # ros2_control Node
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="screen"
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Spawn joint_state_broadcaster
    js_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    # Spawn joint_trajectory_controller
    traj_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
    )

    # Optional: spawn forward position controller but after traj controller
    forward_position_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "-c", "/controller_manager"],
    )

    # Delay joint_state_broadcaster until ros2_control starts
    delay_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=traj_controller_spawner,
            on_exit=[js_broadcaster_spawner]
        )
    )

    # Delay forward_position after JSB
    delay_forward = RegisterEventHandler(
        OnProcessExit(
            target_action=js_broadcaster_spawner,
            on_exit=[forward_position_spawner]
        )
    )

    return LaunchDescription(
        declared_arguments
        + [
            #rviz_node,
            control_node,
            robot_state_publisher,
            traj_controller_spawner,
            delay_jsb,
            delay_forward
        ]
    )

