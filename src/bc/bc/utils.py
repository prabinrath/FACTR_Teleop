
from sensor_msgs.msg import JointState, Image
from geometry_msgs.msg import PoseArray, PoseStamped

from collections import namedtuple
import numpy as np
import cv2

from cv_bridge import CvBridge

def ros2_time_to_ns(ros2_time):
    return int(ros2_time.sec * 1e9 + ros2_time.nanosec)

def create_joint_state_msg(data):
    msg = JointState()
    msg.position = list(map(float, data))
    return msg

def process_msg(msg):
    
    cv_bridge = CvBridge()
    
    if isinstance(msg, JointState):
        data = np.array(msg.position)
    elif isinstance(msg, PoseArray):
        poses = msg.poses
        data = []
        for pose in poses:
            pose = np.array([
                pose.position.x, pose.position.y, pose.position.z,
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
            ])
            data.append(pose)
        data = np.array(data)
    elif isinstance(msg, Image):
        if msg.encoding == "rgb8":
            cv_image = cv_bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        elif msg.encoding == "32FC1":
            cv_image = cv_bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        else:
            raise ValueError(f"Unsupported image encoding {msg.encoding}")
        _, data = cv2.imencode('.jpg', cv_image)
    elif isinstance(msg, PoseStamped):
        data = np.array([
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
            msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w
        ])
    else:
        raise ValueError(f"Unsupported message type {type(msg)}")
    return data