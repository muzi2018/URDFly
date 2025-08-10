import numpy as np
import math
import os


class URDFParser:
    """Class to parse URDF files and extract link and joint information"""

    def __init__(self, urdf_file):
        self.urdf_file = urdf_file
        self.mesh_dir = os.path.dirname(urdf_file)
        self.links = {}
        self.joints = []
        self.parse_urdf()

    def parse_urdf(self):
        """Parse the URDF file to extract links and joints"""
        from xml.etree import ElementTree as ET

        urdf_data = open(self.urdf_file, "r").read()
        root = ET.fromstring(urdf_data)

        # Extract all links
        for link in root.findall("link"):
            visual = link.find("visual")
            if visual is not None:
                geometry = visual.find("geometry")
                if geometry is not None:
                    mesh = geometry.find("mesh")
                    if mesh is not None:
                        filename = mesh.get("filename")
                        # Store the mesh filename and its visual properties
                        self.links[link.get("name")] = {
                            "mesh": filename,
                            "origin": self.parse_origin(visual.find("origin")),
                            "color": self.parse_color(visual.find("material")),
                        }
                    else:
                        self.links[link.get("name")] = None
                else:
                    self.links[link.get("name")] = None
            else:
                self.links[link.get("name")] = None

        # Extract all joints
        for joint in root.findall("joint"):
            joint_data = {
                "name": joint.get("name"),
                "type": joint.get("type"),
                "parent": joint.find("parent").get("link"),
                "child": joint.find("child").get("link"),
                "origin": self.parse_origin(joint.find("origin")),
                "axis": self.parse_axis(joint.find("axis")),
            }
            self.joints.append(joint_data)

    def parse_origin(self, origin_element):
        """Parse origin element to get xyz and rpy"""
        if origin_element is None:
            return {"xyz": [0, 0, 0], "rpy": [0, 0, 0]}

        xyz = list(map(float, origin_element.get("xyz", "0 0 0").split()))
        rpy = list(map(float, origin_element.get("rpy", "0 0 0").split()))
        return {"xyz": xyz, "rpy": rpy}

    def parse_color(self, material_element):
        """Parse color from material element"""
        if material_element is None:
            return [0.5, 0.5, 0.5, 1.0]  # Default gray color

        color = material_element.find("color")
        if color is not None:
            return list(map(float, color.get("rgba", "0.5 0.5 0.5 1.0").split()))
        return [0.5, 0.5, 0.5, 1.0]

    def parse_axis(self, axis_element):
        """Parse axis element"""
        if axis_element is None:
            return [0, 0, 1]
        return list(map(float, axis_element.get("xyz", "0 0 1").split()))

    def compute_transformation(self, rpy, xyz):
        """Compute 4x4 transformation matrix from RPY and XYZ"""
        roll, pitch, yaw = rpy

        # Rotation matrices
        Rx = np.array(
            [
                [1, 0, 0],
                [0, math.cos(roll), -math.sin(roll)],
                [0, math.sin(roll), math.cos(roll)],
            ]
        )

        Ry = np.array(
            [
                [math.cos(pitch), 0, math.sin(pitch)],
                [0, 1, 0],
                [-math.sin(pitch), 0, math.cos(pitch)],
            ]
        )

        Rz = np.array(
            [
                [math.cos(yaw), -math.sin(yaw), 0],
                [math.sin(yaw), math.cos(yaw), 0],
                [0, 0, 1],
            ]
        )

        # Combined rotation
        R = Rz @ Ry @ Rx

        # Transformation matrix
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = xyz

        return T

    def forward_kinematics(self):
        """Compute the transformations for all links"""
        transformations = {"base_link": np.eye(4)}

        for joint in self.joints:
            parent = joint["parent"]
            child = joint["child"]

            # Get parent transformation
            if parent in transformations:
                T_parent = transformations[parent]
            else:
                print(f"Warning: Parent link '{parent}' not found in transformations")
                T_parent = np.eye(4)

            # Compute joint transformation
            T_joint = self.compute_transformation(
                joint["origin"]["rpy"], joint["origin"]["xyz"]
            )

            # Child transformation is parent * joint
            transformations[child] = T_parent @ T_joint

        return transformations

    def get_robot_info(self):
        """Get information about the robot links and their transformations"""
        transformations = self.forward_kinematics()

        link_names = []
        link_mesh_files = []
        link_mesh_transformations = []
        link_frames = []
        link_colors = []

        # Process each link
        for name, T in transformations.items():
            link_info = self.links.get(name)
            if link_info is None or link_info.get("mesh") is None:
                continue

            # Get mesh file path
            mesh_file = link_info["mesh"]
            if not os.path.isabs(mesh_file):
                mesh_file = os.path.join(self.mesh_dir, mesh_file)
            mesh_file = os.path.normpath(mesh_file)

            # Apply the link's visual origin transformation
            T_visual = self.compute_transformation(
                link_info["origin"]["rpy"], link_info["origin"]["xyz"]
            )

            # Combine with the link's transformation
            T_total = T @ T_visual

            link_names.append(name)
            link_mesh_files.append(mesh_file)
            link_mesh_transformations.append(T_total)
            link_frames.append(T)

            link_colors.append(link_info["color"])

        return (
            link_names,
            link_mesh_files,
            link_mesh_transformations,
            link_frames,
            link_colors,
        )
