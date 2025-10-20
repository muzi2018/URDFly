import pybullet as p
import pybullet_data
import os

urdf_path = "/home/wang/URDFly/descriptions/kyon/urdf/kyon.urdf"

print("[INFO] Checking file exists:", os.path.exists(urdf_path))
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # adds basic meshes, if needed

try:
    robot = p.loadURDF(urdf_path, useFixedBase=True)
    print("[SUCCESS] URDF loaded, robot_id =", robot)
except Exception as e:
    print("[ERROR]", e)
