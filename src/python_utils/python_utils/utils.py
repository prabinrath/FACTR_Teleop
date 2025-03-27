import os
import sys

from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerGuardCondition
from rclpy.utilities import timeout_sec_to_nsec


def get_workspace_root():
    workspace_installation_root = os.environ.get('COLCON_PREFIX_PATH', '').split(os.pathsep)[0]
    workspace_root = os.path.abspath(os.path.join(workspace_installation_root, '..'))
    return workspace_root

def add_external_path(path):
    workspace_root = get_workspace_root()
    sys.path.append(os.path.join(workspace_root, path))

