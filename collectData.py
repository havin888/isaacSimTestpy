#opening testingReal file successfully
#getting the right robot and object data from .usd file (by importing pxr)
#added right joints to csv file
#successfully added motion to robot + fetched data

from isaacsim.simulation_app import SimulationApp

simulation_app = SimulationApp({"headless": False})

from pxr import Usd
from omni.isaac.core import World
from omni.usd import get_context
import csv
import os
from isaacsim.core.prims import Articulation
from omni.isaac.dynamic_control import _dynamic_control
import math
import numpy as np


stage_path = "/home/havin/Documents/testingReal.usd"
usd_context = get_context()
usd_context.open_stage(stage_path)
stage = usd_context.get_stage()

if stage is None:
    raise RuntimeError(f"Failed to open USD stage: {stage_path}")
print(f"Successfully opened stage: {stage_path}")

world = World(stage_units_in_meters=1.0)
world.reset()

found_franka = None

for prim in stage.Traverse():
    name = prim.GetName().lower()
    if "franka" in name:
        found_franka = str(prim.GetPath())
print(f"Franka path: {found_franka}")

if not found_franka:
    raise RuntimeError("Could not find Franka prim in the USD stage.")

franka = world.scene.get_object(found_franka)

if franka is None:
    print("Could not retrieve Franka as Isaac object — using prim path directly.")

csv_path = "/home/havin/Documents/franka_gripper_data.csv"
os.makedirs(os.path.dirname(csv_path), exist_ok=True)

franka = Articulation(prim_paths_expr="/World/Franka")

joint_names = franka.joint_names
print("Number of joints:", franka.num_joints)
print("Joint names:", joint_names)
print("Joint order:")
for i, name in enumerate(joint_names):
    print(i, name)

with open(csv_path, "w", newline="") as f:
    writer = csv.writer(f)
    header = ["timestep"] + list(joint_names)
    writer.writerow(header)

    print("Starting simulation with dynamic_control...")

    dc = _dynamic_control.acquire_dynamic_control_interface()

    articulation = dc.get_articulation(found_franka)
    if not articulation:
        raise RuntimeError(f"Could not get articulation at {found_franka}")

    dc.wake_up_articulation(articulation)

    num_dofs = dc.get_articulation_dof_count(articulation)
    print("Number of DOFs (dynamic_control):", num_dofs)

    targets = np.zeros((1, num_dofs), dtype=np.float32)

    for step in range(200):
        world.step(render=True)
        t = step * 0.02
        shoulder_idx = 1
        targets[0, :] = 0.0
        targets[0, shoulder_idx] = 0.5 * math.sin(0.5 * t)

        if num_dofs >= 9:
            left_finger_idx = num_dofs - 2
            right_finger_idx = num_dofs - 1
            grip = 0.02 * math.sin(0.5 * t)
            targets[0, left_finger_idx] = grip
            targets[0, right_finger_idx] = grip

        dc.set_articulation_dof_position_targets(articulation, targets)
        dof_states = dc.get_articulation_dof_states(
            articulation, _dynamic_control.STATE_POS
        )
        positions = dof_states["pos"]
        writer.writerow([step] + positions.tolist())

print(f"Simulation completed, data saved to: {csv_path}")
simulation_app.close()
