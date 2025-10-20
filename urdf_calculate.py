import pybullet as p
import pybullet_data
import re
from typing import Dict, Optional, Tuple

class RobotHeightCalculator:
    """Calculate robot height in PyBullet simulation"""
    
    def __init__(self, urdf_path: str):
        self.urdf_path = urdf_path
        self.robot_id = None
        
    def connect_physics(self, gui: bool = False):
        """Initialize physics engine"""
        mode = p.GUI if gui else p.DIRECT
        p.connect(mode)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
    def load_robot(self) -> int:
        """Load robot URDF"""
        self.robot_id = p.loadURDF(self.urdf_path, useFixedBase=True)
        return self.robot_id
        
    def set_joint_poses(self, joint_angles: Dict[str, float]):
        """Set joint angles using regex patterns"""
        if self.robot_id is None:
            raise ValueError("Robot not loaded")
            
        num_joints = p.getNumJoints(self.robot_id)
        matched_joints = []
        
        for j in range(num_joints):
            joint_info = p.getJointInfo(self.robot_id, j)
            joint_name = joint_info[1].decode("utf-8")
            joint_type = joint_info[2]  # Check if joint is movable
            
            # Skip fixed joints
            if joint_type == p.JOINT_FIXED:
                continue
                
            # Match joint name to patterns
            for pattern, angle in joint_angles.items():
                if re.match(pattern, joint_name):
                    p.resetJointState(self.robot_id, j, angle)
                    matched_joints.append((joint_name, angle))
                    break
        
        return matched_joints
        
    def get_robot_height(self) -> Tuple[float, dict]:
        """Calculate robot height and return detailed info"""
        if self.robot_id is None:
            raise ValueError("Robot not loaded")
            
        # Get base position
        base_pos, base_ori = p.getBasePositionAndOrientation(self.robot_id)
        
        # Find lowest point among all links
        min_z = float("inf")
        lowest_link = None
        link_positions = {}
        
        # Check base link
        if base_pos[2] < min_z:
            min_z = base_pos[2]
            lowest_link = "base"
            
        # Check all other links
        num_joints = p.getNumJoints(self.robot_id)
        for j in range(num_joints):
            joint_info = p.getJointInfo(self.robot_id, j)
            link_name = joint_info[12].decode("utf-8")  # Child link name
            
            link_state = p.getLinkState(self.robot_id, j)
            link_pos = link_state[0]  # World position
            link_positions[link_name] = link_pos
            
            if link_pos[2] < min_z:
                min_z = link_pos[2]
                lowest_link = link_name
        
        height = base_pos[2] - min_z
        
        return height, {
            'base_position': base_pos,
            'lowest_z': min_z,
            'lowest_link': lowest_link,
            'link_positions': link_positions
        }
        
    def get_robot_mass(self) -> Tuple[float, dict]:
        """Calculate total robot mass and return detailed info"""
        if self.robot_id is None:
            raise ValueError("Robot not loaded")
            
        total_mass = 0.0
        link_masses = {}
        
        # Get base link mass
        base_dynamics = p.getDynamicsInfo(self.robot_id, -1)  # -1 for base link
        base_mass = base_dynamics[0]
        total_mass += base_mass
        link_masses["base"] = base_mass
        
        # Get all other link masses
        num_joints = p.getNumJoints(self.robot_id)
        for j in range(num_joints):
            joint_info = p.getJointInfo(self.robot_id, j)
            link_name = joint_info[12].decode("utf-8")  # Child link name
            
            dynamics_info = p.getDynamicsInfo(self.robot_id, j)
            link_mass = dynamics_info[0]
            total_mass += link_mass
            link_masses[link_name] = link_mass
        
        return total_mass, {
            'total_mass': total_mass,
            'link_masses': link_masses,
            'heaviest_link': max(link_masses.items(), key=lambda x: x[1]),
            'lightest_link': min(link_masses.items(), key=lambda x: x[1])
        }
        
    def get_robot_center_of_mass(self) -> Tuple[Tuple[float, float, float], dict]:
        """Calculate robot center of mass"""
        if self.robot_id is None:
            raise ValueError("Robot not loaded")
            
        total_mass = 0.0
        weighted_com = [0.0, 0.0, 0.0]
        link_coms = {}
        
        # Base link
        base_dynamics = p.getDynamicsInfo(self.robot_id, -1)
        base_mass = base_dynamics[0]
        base_com = base_dynamics[3]  # Local COM
        base_pos, base_ori = p.getBasePositionAndOrientation(self.robot_id)
        
        # Transform local COM to world coordinates for base
        world_base_com = p.multiplyTransforms(base_pos, base_ori, base_com, [0,0,0,1])[0]
        
        total_mass += base_mass
        for i in range(3):
            weighted_com[i] += base_mass * world_base_com[i]
        link_coms["base"] = world_base_com
        
        # All other links
        num_joints = p.getNumJoints(self.robot_id)
        for j in range(num_joints):
            joint_info = p.getJointInfo(self.robot_id, j)
            link_name = joint_info[12].decode("utf-8")
            
            dynamics_info = p.getDynamicsInfo(self.robot_id, j)
            link_mass = dynamics_info[0]
            local_com = dynamics_info[3]
            
            # Get link world transform
            link_state = p.getLinkState(self.robot_id, j)
            link_pos = link_state[0]
            link_ori = link_state[1]
            
            # Transform local COM to world coordinates
            world_com = p.multiplyTransforms(link_pos, link_ori, local_com, [0,0,0,1])[0]
            
            total_mass += link_mass
            for i in range(3):
                weighted_com[i] += link_mass * world_com[i]
            link_coms[link_name] = world_com
        
        # Calculate overall COM
        overall_com = tuple(weighted_com[i] / total_mass for i in range(3))
        
        return overall_com, {
            'total_mass': total_mass,
            'center_of_mass': overall_com,
            'link_centers_of_mass': link_coms
        }
        
    def cleanup(self):
        """Disconnect from physics engine"""
        p.disconnect()


def main():
    # Configuration
    # urdf_path = "/home/wang/URDFly/descriptions/urdf0924/urdf/urdf0924.urdf"
    urdf_path = "/home/wang/URDFly/descriptions/kyon/urdf/kyon.urdf"
    # joint_angles = {
    #     r".*_hip_pitch_joint": -0.4,
    #     r".*_hip_roll_joint": 0.0,
    #     r".*_hip_yaw_joint": 0.0,
    #     r".*_knee_joint": 0.8,
    #     r".*_ankle_pitch_joint": -0.4,
    #     r".*_ankle_roll_joint": 0.0,
    #     r"torso_joint": 0.0,
    #     r".*_shoulder_pitch_joint": -0.3,
    #     r"right_shoulder_roll_joint": -0.2,
    #     r"left_shoulder_roll_joint": 0.2,
    #     r".*_shoulder_yaw_joint": 0.0,
    #     r".*_elbow_joint": 0.1,
    #     r".*_wrist_roll_joint": 0.0,
    #     r".*_wrist_pitch_joint": 0.0,
    #     r".*_wrist_yaw_joint": 0.0,
    # }

    joint_angles = {
        # Body base and reference
        r"base_joint": 0.0,
        r"imu_joint": 0.0,
        r"reference": 0.0,

        # Arms
        r"shoulder_yaw_1": 0.0,
        r"shoulder_pitch_1": 0.0,
        r"elbow_pitch_1": 0.0,
        r"wrist_pitch_1": 0.0,
        r"wrist_yaw_1": 0.0,
        r"dagana_1_base_joint": 0.0,
        r"dagana_1_clamp_joint": 0.0,

        r"shoulder_yaw_2": 0.0,
        r"shoulder_pitch_2": 0.0,
        r"elbow_pitch_2": 0.0,
        r"wrist_pitch_2": 0.0,
        r"wrist_yaw_2": 0.0,
        r"dagana_2_base_joint": 0.0,
        r"dagana_2_clamp_joint": 0.0,

        # Legs (4 legs, slightly bent for stability)
        r"hip_roll_1": 0.0,
        r"hip_pitch_1": -0.1,
        r"knee_pitch_1": 0.3,
        r"contact_1_joint": 0.0,

        r"hip_roll_2": 0.0,
        r"hip_pitch_2": -0.1,
        r"knee_pitch_2": 0.3,
        r"contact_2_joint": 0.0,

        r"hip_roll_3": 0.0,
        r"hip_pitch_3": -0.1,
        r"knee_pitch_3": 0.3,
        r"contact_3_joint": 0.0,

        r"hip_roll_4": 0.0,
        r"hip_pitch_4": -0.1,
        r"knee_pitch_4": 0.3,
        r"contact_4_joint": 0.0,
    }

    # Calculate robot properties
    calculator = RobotHeightCalculator(urdf_path)
    # exit()
    
    try:
        calculator.connect_physics()
        calculator.load_robot()
        
        matched_joints = calculator.set_joint_poses(joint_angles)
        print(f"Set angles for {len(matched_joints)} joints:")
        for name, angle in matched_joints:
            print(f"  {name}: {angle:.3f} rad")
        
        # Calculate height
        height, height_details = calculator.get_robot_height()
        print(f"\n=== HEIGHT ANALYSIS ===")
        print(f"Robot height: {height:.3f} m")
        print(f"Base Z position: {height_details['base_position'][2]:.3f} m")
        print(f"Lowest point Z: {height_details['lowest_z']:.3f} m")
        print(f"Lowest link: {height_details['lowest_link']}")
        
        # Calculate mass
        total_mass, mass_details = calculator.get_robot_mass()
        print(f"\n=== MASS ANALYSIS ===")
        print(f"Total robot mass: {total_mass:.3f} kg")
        print(f"Heaviest link: {mass_details['heaviest_link'][0]} ({mass_details['heaviest_link'][1]:.3f} kg)")
        print(f"Lightest link: {mass_details['lightest_link'][0]} ({mass_details['lightest_link'][1]:.3f} kg)")
        
        # Show mass distribution for key components
        print(f"\nKey component masses:")
        for link_name, mass in mass_details['link_masses'].items():
            if mass > 0.1:  # Only show links with significant mass
                print(f"  {link_name}: {mass:.3f} kg")
        
        # Calculate center of mass
        com, com_details = calculator.get_robot_center_of_mass()
        print(f"\n=== CENTER OF MASS ANALYSIS ===")
        print(f"Overall COM: ({com[0]:.3f}, {com[1]:.3f}, {com[2]:.3f}) m")
        print(f"COM height above ground: {com[2] - height_details['lowest_z']:.3f} m")
        print(f"COM height ratio (COM/total_height): {(com[2] - height_details['lowest_z']) / height:.2%}")
        
        # Stability analysis
        base_pos = height_details['base_position']
        com_offset_x = abs(com[0] - base_pos[0])
        com_offset_y = abs(com[1] - base_pos[1])
        print(f"\n=== STABILITY ANALYSIS ===")
        print(f"COM horizontal offset from base:")
        print(f"  X-axis: {com_offset_x:.3f} m")
        print(f"  Y-axis: {com_offset_y:.3f} m")
        print(f"  Total horizontal offset: {(com_offset_x**2 + com_offset_y**2)**0.5:.3f} m")
        
        # Summary
        print(f"\n=== SUMMARY ===")
        print(f"Robot: {urdf_path.split('/')[-1]}")
        print(f"Total mass: {total_mass:.2f} kg")
        print(f"Height: {height:.2f} m")
        print(f"Mass-to-height ratio: {total_mass/height:.2f} kg/m")
        
    finally:
        calculator.cleanup()


if __name__ == "__main__":
    main()