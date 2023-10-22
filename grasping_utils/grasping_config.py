from ament_index_python.packages import get_package_share_directory
import os
import yaml
import pprint

from launch.actions import DeclareLaunchArgument
from rcl_interfaces.msg import ParameterDescriptor

# Get path to the package's share directory
pkg_share_path = get_package_share_directory("grasping_utils")


def get_grasping_config_launch_argument():
    config_choices = []
    for file in os.listdir(os.path.dirname(os.path.realpath(__file__)) + "/../../grasping_utils/config"):
        if file.endswith(".yaml"):
            config_choices.append(file[:-5])

    return DeclareLaunchArgument(
        "grasping_config",
        default_value="default",
        description="Config file name in grasping_utils/config",
        choices=config_choices,
    )


def load_grasping_config(node, config_path, config_keys=None):
    config_path = config_path + ".yaml"
    try:
        with open(os.path.join(pkg_share_path, "config", config_path), "r") as f:
            config_data = yaml.safe_load(f)
            if config_keys is None or config_keys in config_data:
                node.get_logger().info(f"Loaded config file {config_path}")
                return config_data if config_keys is None else config_data[config_keys]
            else:
                node.get_logger().error(f"KeyError: the key {config_keys} does not exist in the config file {config_path}")
                return None
    except FileNotFoundError:
        node.get_logger().error(
            f"FileNotFoundError: the file {config_path} does not exist in config directory {os.path.join(pkg_share_path, 'config')}"
        )
        return None


def custom_pprint(value, indent=0):
    if isinstance(value, list):
        return "[" + ", ".join(map(str, value)) + "]"
    elif isinstance(value, dict):
        out_str = "{\n"
        for k, v in value.items():
            out_str += "  " * (indent + 1) + repr(k) + ": " + custom_pprint(v, indent + 1) + ",\n"
        out_str += "  " * indent + "}"
        return out_str
    else:
        return repr(value)


def print_config(node, **kwargs):
    log_str = f"----- {node.get_name()} configuration -----\n"
    for name, value in kwargs.items():
        if isinstance(value, dict):
            log_str += f"{name}:\n{custom_pprint(value, indent=2)}\n"
            # log_str += f"{name}:\n{pprint.pformat(value, indent=2)}\n"
        else:
            log_str += f"{name}:\n{str(value)}\n"
    log_str += "--------------------"
    node.logger.info(log_str)


# def get_param_from_launch_arg(node, launch_arg):
#     node.declare_parameter(launch_arg.name, launch_arg.default_value, ParameterDescriptor(description=launch_arg.description))
#     return node.get_parameter(launch_arg.name).get_parameter_value().string_value


# TODO overload possible through conf file in ~/
def reachy_check_grasping_config():
    print("here we are")


# rgc = get_reachy_grasping_config()
