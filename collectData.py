#opening testingReal file successfully
#getting the right robot and object data from .usd file (by importing pxr)

from isaacsim.simulation_app import SimulationApp

simulation_app = SimulationApp({"headless": False})

from pxr import Usd
from omni.isaac.core import World
from omni.usd import get_context
import csv
import os

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

with open(csv_path, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["Step", "X", "Y", "Z"]) #need to add all of Franka's joint components

    print("Starting simulation...")
    for step in range(200):
        world.step(render=True)

        if franka and step == 20: # need to work on this part to actually grab the object
            try:
                franka.gripper.close()
            except Exception as e:
                print(f"Could not close gripper at step {step}: {e}")

        if franka and step == 100: #delete this part
            try:
                franka.arm.set_joint_positions(
                    [0.0, -0.5, 0.0, -1.5, 0.0, 1.0, 0.0]
                )
            except Exception as e:
                print(f"Could not move arm: {e}")

        try:
            if franka and hasattr(franka, "end_effector"): #re-write all the steps to connect joints
                pos, _ = franka.end_effector.get_world_pose()
            else:
                prim = stage.GetPrimAtPath(found_franka)
                xform = UsdGeom.Xformable(prim)
                mat = xform.GetLocalTransformation()
                pos = mat.ExtractTranslation()
            writer.writerow([step, pos[0], pos[1], pos[2]])
        except Exception as e:
            print(f"Failed to record position at step {step}: {e}")

print(f"Simulation completed, data saved to: {csv_path}")
simulation_app.close()
