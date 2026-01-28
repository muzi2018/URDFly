import mujoco

# Load model
model = mujoco.MjModel.from_xml_path("/home/wang/workspace/URDFly/descriptions/QuantaQ/urdf/QuantaQv1.5.urdf")

print("---- All link masses ----")

total_mass = 0
# Loop through ALL bodies
for body_id in range(model.nbody):
    # Read the body name
    body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)

    # Some bodies might have no name (returns None)
    if body_name is None:
        body_name = "(unnamed_body_" + str(body_id) + ")"
    
    # Read mass from model.body_mass
    mass = model.body_mass[body_id]
    total_mass += mass
    print(f"{body_id:2d} | {body_name}: {mass}")
print(f"total mass: ", total_mass)



# --------------------------------------------------------
#  ADDITION 1: PRINT ALL JOINT NAMES
# --------------------------------------------------------

print("\n---- All Joint Names ----")
for j in range(model.njnt):
    jname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
    print(f"{j:2d} | {jname}")

# --------------------------------------------------------
#  ADDITION 2: SET JOINT POSITIONS (qpos)
# --------------------------------------------------------

data = mujoco.MjData(model)

# Example: set all joints to zero
data.qpos[:] = 0

# Or: set a specific joint by index (example: joint 3)
# data.qpos[3] = 0.5

# --------------------------------------------------------
#  ADDITION 3: SET A SPECIFIC JOINT BY NAME
# --------------------------------------------------------

def set_joint_by_name(model, data, joint_name, value):
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    if joint_id < 0:
        raise ValueError(f"Joint '{joint_name}' not found!")
    qpos_addr = model.jnt_qposadr[joint_id]  # index in qpos
    data.qpos[qpos_addr] = value

# Example usage:
set_joint_by_name(model, data, "L_hip_pitch_Joint", -0.4)
set_joint_by_name(model, data, "L_knee_pitch_Joint", 0.8)
set_joint_by_name(model, data, "L_ankle_pitch_Joint", -0.4)

set_joint_by_name(model, data, "R_hip_pitch_Joint", -0.4)
set_joint_by_name(model, data, "L_knee_pitch_Joint", 0.8)
set_joint_by_name(model, data, "R_ankle_pitch_Joint", -0.4)

# After modifying qpos, update forward kinematics
mujoco.mj_forward(model, data)

# --------------------------------------------------------
#  HEIGHT MEASUREMENT (same as before)
# --------------------------------------------------------

bodyA = "base_link"
bodyB = "L_ankle_roll_Link"

idA = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, bodyA)
idB = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, bodyB)

posA = data.xpos[idA]
posB = data.xpos[idB]

height = posA[2] - posB[2]

print("\n---- Height Information ----")
print(f"{bodyA} position: {posA}")
print(f"{bodyB} position: {posB}")
print(f"Vertical height difference (z_A - z_B): {height:.6f} m")


