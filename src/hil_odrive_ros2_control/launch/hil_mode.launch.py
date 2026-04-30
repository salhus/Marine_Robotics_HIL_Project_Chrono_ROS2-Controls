from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch.substitutions import FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


HIL_BANNER = """
╔══════════════════════════════════════════════════════════════════════════════════╗
║                        HIL MODE — Hardware-in-the-Loop                         ║
╠══════════════════════════════════════════════════════════════════════════════════╣
║  Real motor + encoder + velocity_pid_node close the control loop.              ║
║  chrono_flap_node evaluates τ_hydro from measured (θ, ω) state (no dynamics    ║
║  integration for control). hil_torque_mixer sums τ_pid + τ_hydro and commands  ║
║  the ODrive.                                                                    ║
║                                                                                 ║
║  Load torque is DISENGAGED by default. To engage:                              ║
║                                                                                 ║
║    ros2 service call /chrono_flap_node/engage_hil \\                           ║
║      std_srvs/srv/SetBool "{data: true}"                                       ║
║                                                                                 ║
║  To disable load contribution from mixer:                                       ║
║    ros2 service call /hil_torque_mixer_node/enable_load \\                     ║
║      std_srvs/srv/SetBool "{data: false}"                                      ║
╚══════════════════════════════════════════════════════════════════════════════════╝
"""


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("hil_odrive_ros2_control"), "config", "controllers.yaml"]
            ),
            description="Path to the ros2_control controllers YAML file.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "enable_visualization",
            default_value="false",
            description="Enable Chrono 3D visualization (requires working Vulkan/GPU).",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "engage_hil_on_start",
            default_value="false",
            description=(
                "If true, automatically engage HIL load torque after 3 s. "
                "Default false for first-launch safety."
            ),
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "enable_rqt",
            default_value="true",
            description="Launch rqt_reconfigure for live parameter editing.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "enable_plotjuggler",
            default_value="true",
            description="Launch PlotJuggler for time-series plotting (install: sudo apt install ros-jazzy-plotjuggler-ros).",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "enable_rviz",
            default_value="true",
            description="Launch RViz2 for 3D visualization.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config",
            default_value="",
            description="Path to an RViz config file (.rviz). Leave empty to open with defaults.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "plotjuggler_layout",
            default_value="",
            description="Path to a PlotJuggler layout file (.xml). Leave empty to open with defaults.",
        )
    )

    controllers_file = LaunchConfiguration("controllers_file")
    enable_visualization = LaunchConfiguration("enable_visualization")
    engage_hil_on_start = LaunchConfiguration("engage_hil_on_start")
    enable_rqt = LaunchConfiguration("enable_rqt")
    enable_plotjuggler = LaunchConfiguration("enable_plotjuggler")
    enable_rviz = LaunchConfiguration("enable_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    plotjuggler_layout = LaunchConfiguration("plotjuggler_layout")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("hil_odrive_ros2_control"), "description", "urdf", "motor.urdf.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    sim_robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("hil_odrive_ros2_control"), "description", "urdf", "motor_sim.urdf.xacro"]
            ),
        ]
    )
    sim_robot_description = {"robot_description": ParameterValue(sim_robot_description_content, value_type=str)}

    # ── Real hardware stack ────────────────────────────────────────────────────────────────────
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_file],
        output="both",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="both",
    )

    sim_robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="sim_robot_state_publisher",
        namespace="sim",
        parameters=[sim_robot_description, {"frame_prefix": "sim/"}],
        remappings=[("joint_states", "/sim_joint_states")],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="both",
    )

    effort_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["motor_effort_controller", "--controller-manager", "/controller_manager"],
        output="both",
    )

    # Identity transform: sim/base_link overlaid exactly on base_link
    static_tf_sim = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "sim/base_link"],
        output="both",
    )

    # ── velocity_pid_node — remapped so its effort output goes to the mixer, not directly to ODrive
    # The mixer (hil_torque_mixer_node) will sum τ_pid + τ_hydro and publish to
    # /motor_effort_controller/commands.
    velocity_pid_node = Node(
        package="odrive_velocity_pid",
        executable="velocity_pid_node",
        name="velocity_pid_node",
        output="both",
        remappings=[
            ("/motor_effort_controller/commands", "/velocity_pid_node/torque_command"),
        ],
    )

    # ── chrono_flap_node in HIL mode: evaluates τ_hydro from measured state, no Chrono integration
    chrono_flap_node = Node(
        package="chrono_flap_sim",
        executable="chrono_flap_node",
        name="chrono_flap_node",
        parameters=[{
            "mode": "hil",
            "enable_visualization": enable_visualization,
        }],
        output="both",
    )

    # ── hil_torque_mixer: sums τ_pid + τ_hydro → /motor_effort_controller/commands
    hil_torque_mixer_node = Node(
        package="hil_torque_mixer",
        executable="hil_torque_mixer_node",
        name="hil_torque_mixer_node",
        output="both",
    )

    # ── Optional: auto-engage HIL after 3 s ──────────────────────────────────────────────────
    auto_engage_action = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2", "service", "call",
                    "/chrono_flap_node/engage_hil",
                    "std_srvs/srv/SetBool",
                    "{data: true}",
                ],
                output="screen",
            )
        ],
        condition=IfCondition(engage_hil_on_start),
    )

    # ── Tool nodes ─────────────────────────────────────────────────────────────────────────────
    rqt_node = Node(
        package="rqt_reconfigure",
        executable="rqt_reconfigure",
        name="rqt_reconfigure",
        output="screen",
        condition=IfCondition(enable_rqt),
    )

    plotjuggler_node = Node(
        package="plotjuggler",
        executable="plotjuggler",
        name="plotjuggler",
        arguments=["-l", plotjuggler_layout],
        output="screen",
        condition=IfCondition(enable_plotjuggler),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
        condition=IfCondition(enable_rviz),
    )

    return LaunchDescription(
        declared_arguments
        + [
            LogInfo(msg=HIL_BANNER),
            control_node,
            robot_state_pub_node,
            sim_robot_state_pub_node,
            joint_state_broadcaster_spawner,
            effort_controller_spawner,
            static_tf_sim,
            velocity_pid_node,
            chrono_flap_node,
            hil_torque_mixer_node,
            auto_engage_action,
            rqt_node,
            plotjuggler_node,
            rviz_node,
        ]
    )
