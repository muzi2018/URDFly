import numpy as np
import math
import os
from anytree import Node, RenderTree



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
        
        # For joint frames
        joint_names = []
        joint_frames = []
        joint_types = []
        joint_axes = []
        joint_parent_links = []
        joint_child_links = []

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
            
        # Process each joint
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
            
            # Joint frame is parent * joint
            joint_frame = T_parent @ T_joint
            
            joint_names.append(joint["name"])
            joint_frames.append(joint_frame)
            joint_types.append(joint["type"])
            joint_axes.append(joint["axis"])
            joint_parent_links.append(parent)
            joint_child_links.append(child)

        return (
            link_names,
            link_mesh_files,
            link_mesh_transformations,
            link_frames,
            link_colors,
            joint_names,
            joint_frames,
            joint_types,
            joint_axes,
            joint_parent_links,
            joint_child_links,
        )
    
    def build_multiple_trees(self):
        """Build multiple kinematic trees if the URDF contains disconnected structures"""
        # Create nodes for all links and joints first
        self.nodes = {}
        
        # Create all link nodes
        for link_name in self.links.keys():
            self.nodes[link_name] = Node(link_name, node_type="link")
            
        # Create joint nodes and establish parent-child relationships
        for joint in self.joints:
            parent_link = joint["parent"]
            child_link = joint["child"]
            joint_name = joint["name"]
            
            # Create joint node with parent being the parent link
            joint_node = Node(
                joint_name,
                parent=self.nodes[parent_link],
                node_type="joint",
                joint_type=joint["type"],
                joint_axis=joint["axis"]
            )
            self.nodes[joint_name] = joint_node
            
            # Set the child link's parent to be this joint
            self.nodes[child_link].parent = joint_node
        
        # Find all root nodes (links with no parent)
        roots = []
        for node in self.nodes.values():
            if node.is_root and node.node_type == "link":
                roots.append(node)
                
        return roots
    
    def identify_chains(self):
        """
        Identify all kinematic chains in the URDF.
        A chain is defined as a path from a root node to a leaf node.
        """
        # Build all trees
        trees = self.build_multiple_trees()
        chains = []
        
        # For each tree, find all paths from root to leaves
        for root in trees:
            # Find all leaf nodes in this tree
            leaves = [node for node in root.descendants if not node.children and node.node_type == "link"]
            
            # For each leaf, create a chain from root to leaf
            for leaf in leaves:
                chain = []
                current = leaf
                
                # Traverse up from leaf to root
                while current:
                    chain.append(current)
                    current = current.parent
                
                # Reverse to get root-to-leaf order
                chain.reverse()
                
                # Create a chain object with relevant information
                chain_info = {
                    "name": f"{root.name}_to_{leaf.name}",
                    "nodes": chain,
                    "links": [node for node in chain if node.node_type == "link"],
                    "joints": [node for node in chain if node.node_type == "joint"],
                    "root": root,
                    "leaf": leaf
                }
                
                chains.append(chain_info)
        
        return chains, trees
    
    def print_chains(self):
        """Print all kinematic chains in the URDF"""
        chains, trees = self.identify_chains()
        
        print(f"Found {len(trees)} kinematic tree(s) and {len(chains)} chain(s)")
        
        for i, tree in enumerate(trees):
            print(f"\nTree {i+1}: Root = {tree.name}")
            
            # Find chains belonging to this tree
            tree_chains = [chain for chain in chains if chain["root"] == tree]
            
            for j, chain in enumerate(tree_chains):
                print(f"  Chain {j+1}: {chain['name']}")
                print(f"    Links: {len(chain['links'])}")
                print(f"    Joints: {len(chain['joints'])}")
                print(f"    Path: ", end="")
                
                # Print the path with alternating links and joints
                for node in chain["nodes"]:
                    if node.node_type == "joint":
                        joint_type = getattr(node, "joint_type", "unknown")
                        print(f"{node.name}({joint_type}) → ", end="")
                    else:
                        if node == chain["nodes"][-1]:  # Last node
                            print(f"{node.name}", end="")
                        else:
                            print(f"{node.name} → ", end="")
                print()
    
    def get_chain_info(self):
        """
        Get detailed information about all kinematic chains.
        Returns a list of dictionaries, each containing information about a chain.
        """
        chains, trees = self.identify_chains()
        chain_info_list = []
        
        # Get transformations for all links
        transformations = self.forward_kinematics()
        
        for chain in chains:
            # Extract link and joint names in order
            link_names = [node.name for node in chain["nodes"] if node.node_type == "link"]
            joint_names = [node.name for node in chain["nodes"] if node.node_type == "joint"]
            
            # Get joint types and axes
            joint_types = []
            joint_axes = []
            
            for joint_name in joint_names:
                joint_node = self.nodes[joint_name]
                joint_types.append(getattr(joint_node, "joint_type", "unknown"))
                joint_axes.append(getattr(joint_node, "joint_axis", [0, 0, 1]))
            
            # Get link transformations
            link_transforms = []
            for link_name in link_names:
                if link_name in transformations:
                    link_transforms.append(transformations[link_name])
                else:
                    link_transforms.append(np.eye(4))
            
            # Create detailed chain info
            detailed_info = {
                "name": chain["name"],
                "root": chain["root"].name,
                "leaf": chain["leaf"].name,
                "link_names": link_names,
                "joint_names": joint_names,
                "joint_types": joint_types,
                "joint_axes": joint_axes,
                "link_transforms": link_transforms,
                "num_links": len(link_names),
                "num_joints": len(joint_names)
            }
            
            chain_info_list.append(detailed_info)
        
        return chain_info_list, trees
            
    

if __name__ == "__main__":
    
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    parser = URDFParser("descriptions/urdf/gx7_test.urdf")
    parser.print_chains()

    chains, _ = parser.get_chain_info()
    
    chain = chains[0]
    
    link_frames = chain["link_transforms"]
    joint_axes = [[0, 0, 1]] + chain["joint_axes"]
    link_names = chain["link_names"]



    # Plot the link frames
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    axis_length = 0.1
    # Plot each link frame
    for name, T, axis in zip(link_names, link_frames, joint_axes):

        # Extract position and orientation
        pos = T[:3, 3]
        rot = axis_length*T[:3, :3]@np.array(axis)

        print(rot)
        # Plot the frame
        ax.quiver(pos[0], pos[1], pos[2], 
                 rot[0], rot[1], rot[2], 
                 color='b')
        
        # 绘制点
        ax.scatter(pos[0], pos[1], pos[2], color='r', marker='o')

        
        # 字体大小
        ax.text(pos[0], pos[1], pos[2], f"{name}", color='g', fontsize=10)
        


    # Set plot limits
    ax.set_xlim([-0.5, 0.5])

    ax.set_ylim([-0.5, 0.5])

    ax.set_zlim([-1, 1])

    # scale the same
    ax.set_box_aspect([1, 1, 1])

    
    # Set labels
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    
    plt.show()
