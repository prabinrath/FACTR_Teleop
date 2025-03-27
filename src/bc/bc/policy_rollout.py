
import numpy as np
from collections import deque
import cv2
import torch
import hydra
import yaml
import rclpy
from pathlib import Path
import datetime
import pickle
from collections import defaultdict
from pynput import keyboard
from data4robotics import misc

from python_utils.utils import add_external_path

add_external_path("scripts")
from data_process_utils import process_decoded_image

from bc.rollout import Rollout

class PolicyRollout(Rollout):
    
    def __init__(self):
        super().__init__()

        self.declare_parameter('save_data', False)
        
        # init agent
        self.init_agent()
        
        # initalize temporal ensemble
        self.pred_horizon = 30
        self.exp_weight = 0.1
        self.act_history = deque(maxlen=self.pred_horizon)
        
        self.dt = 1/30
        self.step_cnt = 0
        self.timer = self.create_timer(self.dt, self.timer_callback)
        
        # obs config
        self.img_chunk = 1
        self.state_chunk = 1
        
        # initialize data dumping
        self.is_save_data = self.get_parameter('save_data').value
        if self.is_save_data:
            current_time = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            self.data_dump_path = Path(self.data_dir) / "eval" / f"eval_{current_time}.pkl"
            self.data_dump_path.parent.mkdir(parents=True, exist_ok=True)
            self.data_dict = defaultdict(list)
            self.listener = keyboard.Listener(on_press=self.on_press_key)
            self.listener.start()
        
    def test_state_obs(self):
        for state_key in self.state_topics:
            if state_key not in self.state_obs.keys():
                self.get_logger().info(f"{state_key} not in state_obs")
                return False
        return True
    
    def test_image_obs(self):
        for image_key in self.camera_topics:
            if image_key not in self.image_obs.keys():
                self.get_logger().info(f"{image_key} not in image_obs")
                return False
        return True
    
    def init_agent(self):
        with open(self.data_dir / "agent_config.yaml", "r") as f:
            config_yaml = f.read()
            agent_config = yaml.safe_load(config_yaml)
        agent = hydra.utils.instantiate(agent_config)
        agent_path = self.data_dir / "latest_ckpt.ckpt"
        load_state_dict = torch.load(agent_path, map_location="cpu")
        misc.GLOBAL_STEP = load_state_dict["global_step"]
        agent.load_state_dict(load_state_dict["model"])
        self.agent = torch.compile(agent.eval().cuda())
        
        step = load_state_dict["global_step"]
        self.get_logger().info(f"Loaded agent from {agent_path} at step {step}.")
    
    def action_chunk_ensemble(self):
        num_actions = len(self.act_history)
        curr_act_preds = np.stack(
            [
                pred_actions[i]
                for (i, pred_actions) in zip(
                    range(num_actions-1, -1, -1), self.act_history
                )
            ]
        )
        # more recent predictions get exponentially *less* weight than older predictions
        weights = np.exp(-self.exp_weight * np.arange(num_actions))
        weights = weights / weights.sum()

        return np.sum(weights[:, None] * curr_act_preds, axis=0)
    
    def prepare_image_obs(self):
        img_obs = {}
        raw_images = {}
        for cam_ind, (cam_topic, raw_img) in enumerate(self.image_obs.items()):
            cam_img = process_decoded_image(raw_img)
            image = cv2.resize(cam_img, (224, 224))
            img_tensor = torch.from_numpy(image[None]).float().permute((0, 3, 1, 2)) / 255
            img_tensor = img_tensor.cuda()
            mean = torch.tensor([0.485, 0.456, 0.406]).view(-1, 1, 1).cuda()
            std = torch.tensor([0.229, 0.224, 0.225]).view(-1, 1, 1).cuda()
            img_tensor = (img_tensor - mean) / std
            img_tensor = img_tensor.unsqueeze(0)
            img_obs[f"cam{cam_ind}"] = img_tensor
            raw_images[cam_topic] = raw_img
        if self.is_save_data:
            self.data_dict["raw_images"].append(raw_images)
        return img_obs
    
    def prepare_state_obs(self):
        if len(self.state_topics) == 0:
            state_tensor = np.zeros((1))
            state_tensor = torch.from_numpy(state_tensor).float().cuda().unsqueeze(0)
            return state_tensor
        
        state_obs = []
        for step in range(1, self.state_chunk+1, 1):
            step_obs = []
            for state_topic in self.state_topics:
                ind = max(0, len(self.state_obs[state_topic])-step)
                step_obs.append(self.state_obs[state_topic][ind])
            step_obs = np.concatenate(step_obs, axis=-1)
            state_obs.append(step_obs)
        state_obs = np.array(state_obs)
        # normalize state
        if self.state_norm_stats is not None:
            mean = np.array(self.state_norm_stats["mean"])
            std = np.array(self.state_norm_stats["std"])
            state_obs = (state_obs - mean) / std
        state_obs = np.concatenate(state_obs, axis=0)
        state_tensor = torch.from_numpy(state_obs).float().cuda().unsqueeze(0)
        return state_tensor

    def get_action(self, img_obs, state_obs):
        pred_action = self.agent.eval().get_actions(img_obs, state_obs)    
        pred_action = pred_action.detach().cpu().numpy()[0]
        self.act_history.append(pred_action)
        cmd_action = self.action_chunk_ensemble()
        return pred_action, cmd_action
    
    def save_data(self):
        self.data_dict["num_steps"] = self.step_cnt
        with open(self.data_dump_path, "wb") as f:
            pickle.dump(self.data_dict, f)
            f.flush()
        self.get_logger().info(f"Saved data to {self.data_dump_path}")
    
    def timer_callback(self):
        
        if not self.test_state_obs() or not self.test_image_obs():
            self.get_logger().info("Waiting for state and image obs")
            return
        
        img_obs = self.prepare_image_obs()
        state_obs = self.prepare_state_obs()
        
        # get action
        pred_action, cmd_action = self.get_action(img_obs, state_obs)
        action_dict = self.decode_action(cmd_action)
        self.send_command(action_dict)
        
        if self.is_save_data:
            self.data_dict["img_obs"].append(img_obs)
            self.data_dict["state_obs"].append(state_obs)
            self.data_dict["pred_actions"].append(pred_action)
            self.data_dict["cmd_actions"].append(action_dict)
        
        self.step_cnt += 1

    def on_press_key(self, key):
        try:
            if key == keyboard.Key.space:
                self.save_data()
            elif key == keyboard.Key.delete:
                self.data_dict = defaultdict(list)
                return
        except Exception as e:
            self.get_logger().info(f"Error saving data: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = PolicyRollout()
    try:
        while rclpy.ok():
            rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received. Shutting down...")
        node.shut_down()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()