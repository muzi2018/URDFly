import pybullet as p
import pybullet_data
import time
import math
import re
from typing import Dict, Optional, Tuple

def connect_pybullet(gui: bool = True):
    """Connect to PyBullet in GUI or DIRECT mode."""
    if gui:
        physicsClient = p.connect(p.GUI)
    else:
        physicsClient = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    return physicsClient

def load_robot(urdf_path: str, base_position=(0, 0, 0), use_fixed_base=True) -> int:
    """Load the URDF robot into the simulation."""
    robot_id = p.loadURDF(urdf_path, basePosition=base_position, useFixedBase=use_fixed_base)
    return robot_id

def get_link_index(robot_id: int, link_name: str) -> Optional[int]:
    """Get the index of a link by name."""
    num_joints = p.getNumJoints(robot_id)
    for i in range(num_joints):
        joint_info = p.getJointInfo(robot_id, i)
        if joint_info[12].decode("utf-8") == link_name:
            return i
    return None

def get_link_position(robot_id: int, link_name: str) -> Optional[Tuple[float, float, float]]:
    """Get the world position of a given link."""
    link_index = get_link_index(robot_id, link_name)
    if link_index is None:
        print(f"[ERROR] Link '{link_name}' not found.")
        return None
    link_state = p.getLinkState(robot_id, link_index)
    return link_state[0]  # linkWorldPosition

def compute_distance(robot_id: int, link_name_1: str, link_name_2: str) -> Optional[float]:
    """Compute Euclidean distance between two links."""
    pos1 = get_link_position(robot_id, link_name_1)
    pos2 = get_link_position(robot_id, link_name_2)
    if pos1 is None or pos2 is None:
        return None
    dist = math.sqrt(sum((a - b) ** 2 for a, b in zip(pos1, pos2)))
    return dist


if __name__ == "__main__":
    # 1️⃣ Connect to PyBullet
    connect_pybullet(gui=True)

    # 2️⃣ Load your robot URDF
    robot_path = "/home/wang/MimicKit/data/assets/urdf0924/urdf/urdf0924.urdf"
    robot_id = load_robot(robot_path)

    # 3️⃣ Let the simulation settle
    for _ in range(100):
        p.stepSimulation()
        time.sleep(1.0 / 240.0)

    # 4️⃣ Compute distance
    link1 = "left_shoulder_pitch_link"
    link2 = "left_ankle_roll_link"
    distance = compute_distance(robot_id, link1, link2)

    if distance is not None:
        print(f"Distance between {link1} and {link2}: {distance:.4f} m")

    # 5️⃣ Keep GUI open
    while True:
        p.stepSimulation()
        time.sleep(1.0 / 240.0)
