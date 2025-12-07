import mujoco

# Load model
model = mujoco.MjModel.from_xml_path("/home/wang/URDFly/descriptions/urdf0924/urdf0924.xml")

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
