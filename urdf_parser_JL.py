import xml.etree.ElementTree as ET

def count_joints_and_links(urdf_file_path):
    # Parse the URDF file
    tree = ET.parse(urdf_file_path)
    root = tree.getroot()

    # Count links
    links = root.findall(".//link")
    num_links = len(links)

    # Count joints
    joints = root.findall(".//joint")
    num_joints = len(joints)

    # Collect joint names
    joint_names = [joint.get("name") for joint in joints]

    # Collect link names
    link_names = [link.get("name") for link in links]

    return num_links, num_joints, joint_names, link_names


if __name__ == "__main__":
    urdf_path = "/home/wang/URDFly/descriptions/kyon/kyon.urdf"
    links, joints, joint_names, link_names = count_joints_and_links(urdf_path)
    print(f"Number of links: {links}")
    print(f"Number of joints: {joints}")
    print("Joint names:")
    for name in joint_names:
        print(f" - {name}")
    print("Link names:")
    for name in link_names:
        print(f" - {name}")
