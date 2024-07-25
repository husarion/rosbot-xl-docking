from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def launch_setup(context):
    aruco_tracker_params_file = LaunchConfiguration("aruco_tracker_params_file").perform(context)
    nav2_docking_server_params_file = LaunchConfiguration("nav2_docking_server_params_file").perform(context)

    return [
        Node(
            package="opennav_docking",
            executable="opennav_docking",
            name="docking_server",
            parameters=[
                nav2_docking_server_params_file,
                {"use_sim_time": True}
            ],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="nav2_docking_lifecycle_manager",
            parameters=[
                {"autostart": True, "node_names": ["docking_server"]},
                {"use_sim_time": True},
            ],
        ),
        Node(
            package="aruco_opencv",
            executable="aruco_tracker_autostart",
            name="aruco_tracker",
            parameters=[
                aruco_tracker_params_file,
                {"use_sim_time": True},
            ],
        ),
        Node(
            # Waits for the appropriate april tag detection and publishes the pose of the detected dock.
            package="rosbot_xl_docking",
            executable="aruco_dock_detector",
            parameters=[
                {
                    "use_sim_time": True
                }
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "aruco_tracker_params_file",
                default_value=str(
                    get_package_share_path("rosbot_xl_docking") / "config" / "aruco_tracker.yaml"
                ),
                description="Path to the aruco tracker parameters file",
            ),
            DeclareLaunchArgument(
                "nav2_docking_server_params_file",
                default_value=str(
                    get_package_share_path("rosbot_xl_docking")
                    / "config"
                    / "nav2_docking_server.yaml"
                ),
                description="Path to the nav2 docking server parameters file",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
