
import numpy as np
import yaml
from pathlib import Path
from cv_bridge import CvBridge
import copy
import functools
from collections import defaultdict, deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image

from bc.utils import create_joint_state_msg
from python_utils.utils import add_external_path

add_external_path("scripts")
from data_process_utils import to_eef, apply_delta_eef

class Rollout(Node):
    
    def __init__(self):
        super().__init__('rollout_node')
        
        self.data_dir = Path(self.declare_parameter('data_dir', "").get_parameter_value().string_value)
        self.init_rollout_config()
        self.state_obs = defaultdict(lambda: deque(maxlen=20))
        self.image_obs = {}
        self.init_messengers()
        self.bridge = CvBridge()
    
    def init_rollout_config(self):
        with open(self.data_dir / "rollout_config.yaml", "r") as f:
            self.rollout_config = yaml.safe_load(f)

        self.action_norm_stats = None
        self.state_norm_stats = None
        if "norm_stats" in self.rollout_config:
            self.action_norm_stats = self.rollout_config["norm_stats"]["action"]
            self.state_norm_stats = self.rollout_config["norm_stats"]["state"]
        
        self.obs_config = self.rollout_config["obs_config"]
        self.state_topics = self.obs_config["state_topics"]
        self.camera_topics = self.obs_config["camera_topics"]
        
        self.action_config = self.rollout_config["action_config"]
        self.action_dim_dict = self.action_config["action_dim_dict"]
        self.action_type = self.action_config["action_type"]
    
    def _joint_state_callback(self, msg: JointState, name: str):
        self.state_obs[name].append(np.array(msg.position))
    
    def _image_callback(self, msg: Image, name: str):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        self.image_obs[name] = image
    
    def init_messengers(self):
        self.get_logger().info(f"Initializing messengers")
        self.obs_subscribers = {}
        subcribed_topics = copy.deepcopy(self.state_topics)
        for state_topic in subcribed_topics:
            self.obs_subscribers[state_topic] = self.create_subscription(
                JointState, state_topic, 
                functools.partial(self._joint_state_callback, name=state_topic),
                1,
            )
        for camera_topic in self.camera_topics:
            self.obs_subscribers[camera_topic] = self.create_subscription(
                Image, camera_topic, 
                functools.partial(self._image_callback, name=camera_topic),
                1,
            )
        
        self.action_publishers = {}
        for action_topic in self.action_dim_dict.keys():
            self.action_publishers[action_topic] = self.create_publisher(JointState, action_topic, 10)
    
    def decode_action(self, action):
        
        # unnormalize action
        if self.action_norm_stats is not None:
            mean = np.array(self.action_norm_stats["mean"])
            std = np.array(self.action_norm_stats["std"])
            action = action * std + mean
        
        # decode action
        dim_pointer = 0
        action_dict = {}
        for action_key, action_dim in self.action_dim_dict.items():
            action_dict[action_key] = action[dim_pointer:dim_pointer+action_dim]
            dim_pointer += action_dim
        
        return action_dict
        
    def send_command(self, action_dict: dict):
        for action_key, action_value in action_dict.items():
            self.action_publishers[action_key].publish(create_joint_state_msg(action_value))
        
    
def main(args=None):
    rclpy.init(args=args)
    node = Rollout()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()