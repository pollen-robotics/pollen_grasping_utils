import numpy as np
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import Pose
from pollen_msgs.msg import GraspPose
from rclpy.duration import Duration
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray


def get_visualization_marker_msg(
    header: Header,
    id: int,
    type: str,
    action: str,
    pose: tuple = (0, 0, 0),
    orientation: tuple = (0, 0, 0, 1),
    scale: tuple = (0.1, 0.1, 0.1),
    color: tuple = (1.0, 0, 0, 1.0),
    lifetime: float = 0,
):
    m = Marker()
    m.header = header
    m.id = id

    try:
        m.type = getattr(Marker, type.upper())
    except KeyError:
        print(
            "Marker type should be in ['arrow', 'cube', 'sphere', 'cylinder', 'line_strip', 'line_list', \
            'cube_list', 'sphere_list', 'points', 'text_view_facing', 'mesh_resource', 'triangle_list']"
        )
        return None
    try:
        m.action = getattr(Marker, action.upper())
    except KeyError:
        print("Marker action should be in ['add', , 'modify', 'delete', 'deleteall']")
        return None

    m.pose.position.x = float(pose[0])
    m.pose.position.y = float(pose[1])
    m.pose.position.z = float(pose[2])

    m.pose.orientation.x = float(orientation[0])
    m.pose.orientation.y = float(orientation[1])
    m.pose.orientation.z = float(orientation[2])
    m.pose.orientation.w = float(orientation[3])

    m.scale.x = float(scale[0])
    m.scale.y = float(scale[1])
    m.scale.z = float(scale[2])

    m.color.r = float(color[0])
    m.color.g = float(color[1])
    m.color.b = float(color[2])
    m.color.a = float(color[3])

    m.lifetime = Duration(seconds=lifetime).to_msg()

    return m


def get_grasp_marker(
    header: Header,
    grasp_pose: Pose,
    tip_length: float = 0.1,
    width: int = 40,
    marker_id: int = 0,
    score: float = 1.0,
    color: tuple = (1.0, 0, 0, 1.0),
    lifetime: float = 5.0,
):
    """Generate a fork like marker to visualize a grasp pose.

    The fork is composed of a cylinder representing the base of the gripper, a cylinder representing the tip of the
    gripper and two cylinders representing the two fingers of the gripper.
    """
    x = grasp_pose.position.x
    y = grasp_pose.position.y
    z = grasp_pose.position.z

    # Calculating the direction vectors in the base of the grasp pose
    orientation = grasp_pose.orientation

    grasp_quaternion = R.from_quat([orientation.x, orientation.y, orientation.z, orientation.w])

    # Reversing the 90° rotation that is needed to match the gripper frame
    grasp_quaternion = (grasp_quaternion * R.from_euler("xyz", [0.0, 180.0, 90.0], degrees=True)).as_quat()

    # Normalize the quaternion to make sure it represents a direction
    direction = grasp_quaternion / np.linalg.norm(grasp_quaternion)

    # direction = grasp_quaternion.normalised
    direction_vector_x = R.from_quat(direction).apply(np.array([1.0, 0.0, 0.0]))
    direction_vector_y = R.from_quat(direction).apply(np.array([0.0, 1.0, 0.0]))
    direction_vector_z = R.from_quat(direction).apply(np.array([0.0, 0.0, 1.0]))

    marker_color = (color[0] * (score * score), color[1] * (score * score), color[2] * (score * score), 1.0)

    marker_tip = get_visualization_marker_msg(
        header=header,
        id=marker_id,
        type="cylinder",
        action="add",
        pose=np.array([x, y, z]) - tip_length / 2 * direction_vector_z,
        orientation=grasp_quaternion,
        scale=(0.01, 0.01, tip_length),
        color=marker_color,
        lifetime=lifetime,
    )

    base_quaternion = (R.from_quat(grasp_quaternion) * R.from_euler("xyz", [0.0, 90.0, 0.0], degrees=True)).as_quat()

    marker_base = get_visualization_marker_msg(
        header=header,
        id=1 + marker_id,
        type="cylinder",
        action="add",
        pose=np.array([x, y, z]),  # Going "backwards" from the grasp point along the grasp direction
        orientation=base_quaternion,
        scale=(0.01, 0.01, 2.0 * width / 1000.0),
        color=marker_color,
        lifetime=lifetime,
    )

    marker_grip1 = get_visualization_marker_msg(
        header=header,
        id=2 + marker_id,
        type="cylinder",
        action="add",
        pose=np.array([x, y, z]) + tip_length / 4 * direction_vector_z + width / 1000.0 * direction_vector_x,
        orientation=grasp_quaternion,
        scale=(0.01, 0.01, tip_length / 2),
        color=marker_color,
        lifetime=lifetime,
    )

    marker_grip2 = get_visualization_marker_msg(
        header=header,
        id=3 + marker_id,
        type="cylinder",
        action="add",
        pose=np.array([x, y, z]) + tip_length / 4 * direction_vector_z - width / 1000.0 * direction_vector_x,
        orientation=grasp_quaternion,
        scale=(0.01, 0.01, tip_length / 2),
        color=marker_color,
        lifetime=lifetime,
    )

    marker_array = MarkerArray(markers=[marker_tip, marker_base, marker_grip1, marker_grip2])

    return marker_array


def get_pose_msg_from_euler(
    position: tuple = (0, 0, 0),  # (x, y, z)
    euler_angles: tuple = (0, 0, 0),  # (roll, pitch, yaw)
):
    pose = Pose()
    pose.position.x = float(position[0])
    pose.position.y = float(position[1])
    pose.position.z = float(position[2])

    q = R.from_euler("xyz", euler_angles, degrees=False).as_quat()
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]

    return pose


def get_pose_msg_from_quaternion(
    position: tuple = (0, 0, 0),  # (x, y, z)
    quaternion: tuple = (0, 0, 0, 1),  # (x, y, z, w)
):
    pose = Pose()
    pose.position.x = float(position[0])
    pose.position.y = float(position[1])
    pose.position.z = float(position[2])

    pose.orientation.x = float(quaternion[0])
    pose.orientation.y = float(quaternion[1])
    pose.orientation.z = float(quaternion[2])
    pose.orientation.w = float(quaternion[3])

    return pose


def get_pose_msg_from_homogeneous_matrix(homogeneous_matrix: list):
    pose = Pose()
    pose.position.x = homogeneous_matrix[0][3]
    pose.position.y = homogeneous_matrix[1][3]
    pose.position.z = homogeneous_matrix[2][3]

    q = R.from_matrix(homogeneous_matrix[:3, :3]).as_quat()
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]

    return pose


def get_grasp_pose_msg_from_euler(
    class_id: str = "None",
    grasp_id: str = "None",
    header: Header = Header(),
    position: tuple = (0.0, 0.0, 0.0),  # (x, y, z)
    euler_angles: tuple = (0.0, 0.0, 0.0),  # (roll, pitch, yaw)
    grasp_quality: float = 0.0,
    grasp_opening: float = 0.0,
    margins: list[float] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
):
    g = GraspPose()
    g.class_id = class_id
    g.grasp_id = grasp_id
    g.grasp_pose.header = header

    g.grasp_pose.pose.position.x = float(position[0])
    g.grasp_pose.pose.position.y = float(position[1])
    g.grasp_pose.pose.position.z = float(position[2])

    # The rotation along y is there to match the gripper frame. TODO test this in weird angles.
    r = R.from_euler("xyz", euler_angles, degrees=False)
    q = r.as_quat()
    g.grasp_pose.pose.orientation.x = q[0]
    g.grasp_pose.pose.orientation.y = q[1]
    g.grasp_pose.pose.orientation.z = q[2]
    g.grasp_pose.pose.orientation.w = q[3]

    g.grasp_quality = float(grasp_quality)
    g.grasp_opening = float(grasp_opening)
    g.margins = margins

    return g


def get_grasp_pose_msg_from_quaternion(
    class_id: str = "None",
    grasp_id: str = "None",
    header: Header = Header(),
    position: tuple = (0.0, 0.0, 0.0),  # (x, y, z)
    quaternion: tuple = (0.0, 0.0, 0.0, 0.0),  # (qx, qy, qz, qw)
    grasp_quality: float = 0.0,
    grasp_opening: float = 0.0,
    margins: list[float] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
):
    g = GraspPose()
    g.class_id = class_id
    g.grasp_id = grasp_id
    g.grasp_pose.header = header

    g.grasp_pose.pose.position.x = float(position[0])
    g.grasp_pose.pose.position.y = float(position[1])
    g.grasp_pose.pose.position.z = float(position[2])

    g.grasp_pose.pose.orientation.x = float(quaternion[0])
    g.grasp_pose.pose.orientation.y = float(quaternion[1])
    g.grasp_pose.pose.orientation.z = float(quaternion[2])
    g.grasp_pose.pose.orientation.w = float(quaternion[3])

    g.grasp_quality = float(grasp_quality)
    g.grasp_opening = float(grasp_opening)
    g.margins = margins

    return g


def get_grasp_pose_msg_from_homogeneous_matrix(
    class_id: str = "None",
    grasp_id: str = "None",
    header: Header = Header(),
    homogeneous_matrix: list = np.eye(4),
    grasp_quality: float = 0.0,
    grasp_opening: float = 0.0,
    margins: list[float] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
):
    g = GraspPose()
    g.class_id = class_id
    g.grasp_id = grasp_id
    g.grasp_pose.header = header

    g.grasp_pose.pose.position.x = float(homogeneous_matrix[0][3])
    g.grasp_pose.pose.position.y = float(homogeneous_matrix[1][3])
    g.grasp_pose.pose.position.z = float(homogeneous_matrix[2][3])

    q = R.from_matrix(homogeneous_matrix[:3, :3]).as_quat()

    g.grasp_pose.pose.orientation.x = float(q[0])
    g.grasp_pose.pose.orientation.y = float(q[1])
    g.grasp_pose.pose.orientation.z = float(q[2])
    g.grasp_pose.pose.orientation.w = float(q[3])

    g.grasp_quality = float(grasp_quality)
    g.grasp_opening = float(grasp_opening)
    g.margins = margins

    return g


def get_homogeneous_matrix_msg_from_euler(
    position: tuple = (0, 0, 0),  # (x, y, z)
    euler_angles: tuple = (0, 0, 0),  # (roll, pitch, yaw)
    degrees: bool = False,
):
    homogeneous_matrix = np.eye(4)
    homogeneous_matrix[:3, :3] = R.from_euler("xyz", euler_angles, degrees=degrees).as_matrix()
    homogeneous_matrix[:3, 3] = position
    return homogeneous_matrix
