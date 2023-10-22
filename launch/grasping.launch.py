from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

DEFAULT_NONE_VALUE = "default_none_value"

luxonis_default_params = {
    "topics": {
        "color_topic": "/oak/rgb/image_raw",
        "depth_topic": "/oak/stereo/converted_depth",
        "det2d_topic": "/yolo/detections2d",
        "det3d_topic": "/oak/nn/spatial_detections",
    },
    "camera_info_topic": "/oak/rgb/camera_info",
}

simulation_default_params = {
    "topics": {
        "color_topic": "/depth_camera/color/image_raw",
        "depth_topic": "/depth_camera/depth/image_rect_raw",
        "det2d_topic": "/yolo/detections2d",
        "det3d_topic": "/yolo/detections3d",
    },
    "camera_info_topic": "/oak/rgb/camera_info",
}


def launch_setup(context, *args, **kwargs):
    simulation_mode = context.launch_configurations["simulation"] == "true"

    get_object_detection_args = simulation_default_params["topics"] if simulation_mode else luxonis_default_params["topics"]
    camera_info_topic = (
        simulation_default_params["camera_info_topic"] if simulation_mode else luxonis_default_params["camera_info_topic"]
    )
    if context.launch_configurations["color_topic"] != DEFAULT_NONE_VALUE:
        get_object_detection_args["color_topic"] = context.launch_configurations["color_topic"]
    if context.launch_configurations["depth_topic"] != DEFAULT_NONE_VALUE:
        get_object_detection_args["depth_topic"] = context.launch_configurations["depth_topic"]
    if context.launch_configurations["det2d_topic"] != DEFAULT_NONE_VALUE:
        get_object_detection_args["det2d_topic"] = context.launch_configurations["det2d_topic"]
    if context.launch_configurations["det3d_topic"] != DEFAULT_NONE_VALUE:
        get_object_detection_args["det3d_topic"] = context.launch_configurations["det3d_topic"]

    grasp_pose_generator_nodes = []

    if "simple_grasp_pose" in context.launch_configurations["grasp_pose_generator"]:
        grasp_pose_generator_nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [FindPackageShare("simple_grasp_pose"), "/launch", "/simple_grasp_pose.launch.py"]
                )
            )
        )
    if "graspcnn" in context.launch_configurations["grasp_pose_generator"]:
        grasp_pose_generator_nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([FindPackageShare("grasp_cnn"), "/launch", "/ggcnn2.launch.py"]),
                launch_arguments={
                    "network_file": "ggcnn.tjm",
                    "nb_grasps": "5",
                    "camera_info_topic": camera_info_topic,
                }.items(),
            )
        )

    get_object_detection_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare("spatial_yolo"), "/launch", "/get_object_detection.launch.py"]),
        launch_arguments=get_object_detection_args.items(),
    )

    spatial_yolo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare("spatial_yolo"), "/launch", "/spatial_yolo.launch.py"]),
        condition=IfCondition(LaunchConfiguration("simulation")),
    )

    return [*grasp_pose_generator_nodes, get_object_detection_node, spatial_yolo]


def generate_launch_description():
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [FindPackageShare("reachable_poses"), "/launch", "/reachable_grasp_triplet.launch.py"]
                )
            ),
            # TODO better use separate boolean arg so that includelaunchdescription may be done wihtout opaque function and thus allow natural import of arguments ?
            DeclareLaunchArgument(
                "grasp_pose_generator",
                default_value="simple_grasp_pose",
                description="2D detection topic",
                choices=["simple_grasp_pose", "graspcnn", "simple_grasp_pose,graspcnn"],  # TODO auto generate such list
            ),
            DeclareLaunchArgument(
                "simulation",
                default_value="false",
                description="Start grasping stack in simulation, with simulation config and offsets",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "color_topic",
                default_value=DEFAULT_NONE_VALUE,
                description="Color image topic, default value depends on simulation arg",
            ),
            DeclareLaunchArgument(
                "depth_topic",
                default_value=DEFAULT_NONE_VALUE,
                description="Depth image topic, default value depends on simulation arg",
            ),
            DeclareLaunchArgument(
                "det2d_topic",
                default_value=DEFAULT_NONE_VALUE,
                description="2D detection topic, default value depends on simulation arg",
            ),
            DeclareLaunchArgument(
                "det3d_topic",
                default_value=DEFAULT_NONE_VALUE,
                description="3D detection topic, default value depends on simulation arg",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
