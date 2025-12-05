#opening testingReal file successfully
#getting the right robot and object data from .usd file (by importing pxr)

from isaacsim.simulation_app import SimulationApp

simulation_app = SimulationApp({"headless": False})

from pxr import Usd
from omni.isaac.core import World
from omni.usd import get_context
import csv
import os
import numpy as np
from isaacsim.core.prims import Articulation

exts."omni.kit.window.script_editor".snippetFolders = [ #folder to look for snippets
    "${kit}/snippets",
    "C:/home/havin/Documents/testingReal.usd"
]

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
found_cube = None

for prim in stage.Traverse():
    name = prim.GetName().lower()
    if "franka" in name:
        found_franka = str(prim.GetPath())
    elif "cube" in name:
        found_cube = str(prim.GetPath())

print(f"Franka path: {found_franka}")
print(f"Cube path: {found_cube}")

if not found_franka:
    raise RuntimeError("Could not find Franka prim in the USD stage.")
if not found_cube:
    raise RuntimeError("Could not find Cube prim in the USD stage.")

franka = world.scene.get_object(found_franka)
cube = world.scene.get_object(found_cube)

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

    print("Starting simulation...")
    for step in range(200): #deleting this part, will work on it
        world.step(render=True)
        try:
            if franka:
                if step >= 20 and step < 60:
                    q = np.array(franka.get_joint_positions()).reshape(-1)

                    q[-2] += 0.0005
                    q[-1] -= 0.0005

                    franka.set_joint_positions(q.tolist())

                if step >= 100:
                    q = np.array(franka.get_joint_positions()).reshape(-1)
                    q[1] -= 0.001
                    franka.set_joint_positions(q.tolist())
        except Exception as e:
            print(f"Motion command failed at step {step}: {e}")
    try:
        q = np.array(franka.get_joint_positions()).reshape(-1)
        row = [step] + q.tolist()
        writer.writerow(row)
    except Exception as e:
        print(f"Failed to record joint positions at step {step}: {e}")

print(f"Simulation completed, data saved to: {csv_path}")
simulation_app.close()
