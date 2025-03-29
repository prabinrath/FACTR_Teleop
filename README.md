
<h1> Force-Feedback Teleoperation (FACTR teleop) </h1>



#### [Jason Jingzhou Liu](https://jasonjzliu.com)<sup>\*</sup>, [Yulong Li](https://yulongli42.github.io)<sup>\*</sup>, [Kenneth Shaw](https://kennyshaw.net), [Tony Tao](https://tony-tao.com), [Ruslan Salakhutdinov](https://www.cs.cmu.edu/~rsalakhu/), [Deepak Pathak](https://www.cs.cmu.edu/~dpathak/)
_Carnegie Mellon University_

[Project Page](https://jasonjzliu.com/factr/) | [arXiV](https://arxiv.org/abs/2502.17432)

<h1> </h1>
<img src="assets/main_teaser.jpg" alt="teaser" width="750"/>

<br>

## Catalog
- [Installation](#installation)
- [Data Collection](#data-collection)
- [Policy Rollout](#policy-rollout)
- [Citation](#citation)
  
<!-- ## Installation

Our repository requires ROS2. If not already installed, follow [ROS2 Installation Instructions](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html) to install ROS2.
We provide the following ROS2 packages
- factr_teleop
- bc
- cameras
- python_utils

These packages are located at 
<repo_root>/src

These packages must exist inside of a ROS2 workspace. If you do not have an existing ROS2 Workspace then create a [workspace](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html).

Copy the four provided packages into your workspace and build the workspace. See [ROS2 Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html) for further help and then rebuild your workspace using colcon.

Then install the following python packages:
pip install zmq
python -m pip install pin

Then cd into <repo_root>/src/factr_teleop/factr_teleop/dynamixel, run
pip install -e python -->


## Installation

This repository requires **ROS 2**.
If you have nott installed ROS 2 yet, follow the official [ROS 2 installation guide](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html).

### Provided ROS 2 Packages

The following ROS 2 packages are included in this repository:

- `factr_teleop`
- `bc`
- `cameras`
- `python_utils`

These packages are located in:

```
<repo_root>/src
```

### ROS 2 Workspace Setup

These packages must reside within a **ROS 2 workspace**. If you do not already have one, create a workspace by following the [ROS 2 workspace tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html).

Then:

1. Copy the four provided packages into your workspace's `src/` directory.
2. Build the workspace:
   ```bash
   colcon build --symlink-install
   ```

> For more guidance, refer to the [ROS 2 Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html).

### Additional Python Dependencies

Install the following Python packages:

```bash
pip install zmq
python -m pip install pin
```

Finally, navigate to the Dynamixel submodule and install it via:

```bash
cd <repo_root>/src/factr_teleop/factr_teleop/dynamixel
pip install -e python
```



## Data Collection
We provide instructions and sample data collection scripts in ROS2. You might need your custom nodes for robots and sensors to run the system. In our case, the collected data is saved in following format:
### Data Structure
Each trajectory is saved as a separate pickle file. Each pickle file contains a dictionary with the following structure:
```
trajectory.pkl
├── "data" : dict
│   ├── "topic_name_1" : list[data_points]
│   ├── "topic_name_2" : list[data_points]
│   └── ...
└── "timestamps" : dict
    ├── "topic_name_1" : list[timestamps]
    ├── "topic_name_2" : list[timestamps]
    └── ...
```
### Key Components:

- **data**: A dictionary where:
  - Keys are the data source names (ROS topic names in our implementation)
  - Values are lists containing the actual data points (low-dimensional states or images)

- **timestamps**: A dictionary where:
  - Keys are the same data source names as in the "data" dictionary
  - Values are lists containing the timestamps when each corresponding data point was recorded

*Note*: Different data sources may log at different frequencies, resulting in varying list lengths across data sources. The timestamps are crucial for properly aligning and post-processing the data.
While ROS provides synchronization APIs, we chose to record raw timestamps and perform post-processing to allow for greater flexibility in data analysis and alignment.
```python
# Example of a trajectory structure
{
    "data": {
        "/camera/rgb/image_raw": [image1, image2, ...],
        "/joint_states": [state1, state2, ...],
        "/robot/end_effector_pose": [pose1, pose2, ...]
    },
    "timestamps": {
        "/camera/rgb/image_raw": [1615420323.45, 1615420323.55, ...],
        "/joint_states": [1615420323.40, 1615420323.50, ...],
        "/robot/end_effector_pose": [1615420323.42, 1615420323.52, ...]
    }
}
```

### Data Processing and Training
Please check for detailed instructions in our [factr](insert_link) repo.

## Policy Rollout
We provide a sample rollout script in ROS2. Again, custom nodes for robots and sensors need to be implemented. In our case, the rollout launch file could be called as follows: 
```bash
ros2 launch factr_teleop/launch/rollout.py
```
Please checkout [rollout.py](factr_teleop/launch/rollout.py) for details about configurations.

## Citation
If you find this codebase useful, feel free to cite our work!
<div style="display:flex;">
<div>

```bibtex
@article{factr,
  title={FACTR: Force-Attending Curriculum Training for Contact-Rich Policy Learning},
  author={Liu, Jason Jingzhou and Li, Yulong and Shaw, Kenneth and Tao, Tony and Salakhutdinov, Ruslan and Pathak, Deepak},
  journal={arXiv preprint arXiv:2502.17432},
  year={2025}
}
```
